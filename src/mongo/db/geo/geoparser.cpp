/**
 *    Copyright (C) 2012 10gen Inc.
 *
 *    This program is free software: you can redistribute it and/or  modify
 *    it under the terms of the GNU Affero General Public License, version 3,
 *    as published by the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Affero General Public License for more details.
 *
 *    You should have received a copy of the GNU Affero General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *    As a special exception, the copyright holders give permission to link the
 *    code of portions of this program with the OpenSSL library under certain
 *    conditions as described in each individual source file and distribute
 *    linked combinations including the program with the OpenSSL library. You
 *    must comply with the GNU Affero General Public License in all respects for
 *    all of the code used other than as permitted herein. If you modify file(s)
 *    with this exception, you may extend this exception to your version of the
 *    file(s), but you are not obligated to do so. If you do not wish to do so,
 *    delete this exception statement from your version. If you delete this
 *    exception statement from all source files in the program, then also delete
 *    it in the license file.
 */

#include "mongo/db/geo/geoparser.h"

#include <string>
#include <vector>

#include "mongo/db/geo/shapes.h"
#include "mongo/db/jsobj.h"
#include "mongo/util/log.h"
#include "mongo/util/mongoutils/str.h"
#include "third_party/s2/s2polygonbuilder.h"

namespace mongo {

    // This field must be present, and...
    static const string GEOJSON_TYPE = "type";
    // Have one of these values:
    static const string GEOJSON_TYPE_POINT = "Point";
    static const string GEOJSON_TYPE_LINESTRING = "LineString";
    static const string GEOJSON_TYPE_POLYGON = "Polygon";
    static const string GEOJSON_TYPE_MULTI_POINT = "MultiPoint";
    static const string GEOJSON_TYPE_MULTI_LINESTRING = "MultiLineString";
    static const string GEOJSON_TYPE_MULTI_POLYGON = "MultiPolygon";
    static const string GEOJSON_TYPE_GEOMETRY_COLLECTION = "GeometryCollection";
    // This field must also be present.  The value depends on the type.
    static const string GEOJSON_COORDINATES = "coordinates";
    static const string GEOJSON_GEOMETRIES = "geometries";

    // Coordinate System Reference
    // see http://portal.opengeospatial.org/files/?artifact_id=24045
    // and http://spatialreference.org/ref/epsg/4326/
    // and http://www.geojson.org/geojson-spec.html#named-crs
    static const string CRS_CRS84 = "urn:ogc:def:crs:OGC:1.3:CRS84";
    static const string CRS_EPSG_4326 = "EPSG:4326";
    static const string CRS_STRICT_WINDING = "urn:mongodb:strictwindingcrs:EPSG:4326";

    // XXX Return better errors for all bad values
    static const Status BAD_VALUE_STATUS(ErrorCodes::BadValue, "");

    static Status newParseFlatPoint(const BSONElement &elem, Point *out, bool allowAddlFields = false) {
        if (!elem.isABSONObj()) return BAD_VALUE_STATUS;
        BSONObjIterator it(elem.Obj());
        BSONElement x = it.next();
        if (!x.isNumber()) { return BAD_VALUE_STATUS; }
        BSONElement y = it.next();
        if (!y.isNumber()) { return BAD_VALUE_STATUS; }
        if (!allowAddlFields && it.more()) { return BAD_VALUE_STATUS; }
        out->x = x.number();
        out->y = y.number();
        return Status::OK();
    }

    Status GeoParser::newParseLegacyPoint(const BSONElement &elem, PointWithCRS *out, bool allowAddlFields) {
        Status status = newParseFlatPoint(elem, &out->oldPoint, allowAddlFields);
        if (status.isOK()) { out->crs = FLAT; }
        return status;
    }

    static S2Point coordToPoint(double lng, double lat) {
        // We don't rely on drem to clean up non-sane points.  We just don't let them become
        // spherical.
        verify(isValidLngLat(lng, lat));
        // Note that it's (lat, lng) for S2 but (lng, lat) for MongoDB.
        S2LatLng ll = S2LatLng::FromDegrees(lat, lng).Normalized();
        // This shouldn't happen since we should only have valid lng/lats.
        if (!ll.is_valid()) {
            stringstream ss;
            ss << "coords invalid after normalization, lng = " << lng << " lat = " << lat << endl;
            uasserted(17125, ss.str());
        }
        return ll.ToPoint();
    }

    static Status newParseGeoJSONCoodinate(const BSONElement& elem, S2Point* out) {
        if (Array != elem.type()) { return BAD_VALUE_STATUS; }
        Point p;
        // Check the object has and only has 2 numbers.
        Status status = newParseFlatPoint(elem, &p);
        if (!status.isOK()) return status;
        if (!isValidLngLat(p.x, p.y)) { return BAD_VALUE_STATUS; }
        *out = coordToPoint(p.x, p.y);
        return Status::OK();
    }

    // "coordinates": [ [100.0, 0.0], [101.0, 1.0] ]
    static Status newParseArrayOfCoodinates(const BSONElement& elem, vector<S2Point>* out) {
        if (Array != elem.type()) { return BAD_VALUE_STATUS; }
        BSONObjIterator it(elem.Obj());
        // Iterate all coordinates in array
        while (it.more()) {
            S2Point p;
            Status status = newParseGeoJSONCoodinate(it.next(), &p);
            if (!status.isOK()) return status;
            out->push_back(p);
        }
        return Status::OK();
    }

    static void eraseDuplicatePoints(vector<S2Point>* vertices) {
        for (size_t i = 1; i < vertices->size(); ++i) {
            if ((*vertices)[i - 1] == (*vertices)[i]) {
                vertices->erase(vertices->begin() + i);
                // We could have > 2 adjacent identical vertices, and must examine i again.
                --i;
            }
        }
    }

    static Status newIsLoopClosed(const vector<S2Point>& loop) {
        if (loop.empty() || loop[0] != loop[loop.size() - 1]) return BAD_VALUE_STATUS;
        return Status::OK();
    }

    static Status parseGeoJSONPolygonCoordinates(const BSONElement& elem, S2Polygon *out) {
        if (Array != elem.type()) { return BAD_VALUE_STATUS; }

        OwnedPointerVector<S2Loop> loops;
        Status status = Status::OK();

        BSONObjIterator it(elem.Obj());
        // Iterate all loops of the polygon.
        while (it.more()) {
            // Parse the array of vertices of a loop.
            BSONElement coordinateElt = it.next();
            vector<S2Point> points;
            status = newParseArrayOfCoodinates(coordinateElt, &points);
            if (!status.isOK()) return status;

            // Check if the loop is closed.
            status = newIsLoopClosed(points);
            if (!status.isOK()) return status;

            eraseDuplicatePoints(&points);
            // Drop the duplicated last point.
            points.resize(points.size() - 1);

            S2Loop* loop = new S2Loop(points);
            loops.push_back(loop);

            // Check whether this loop is valid.
            // 1. At least 3 vertices.
            // 2. All vertices must be unit length. Guaranteed by parsePoints().
            // 3. Loops are not allowed to have any duplicate vertices.
            // 4. Non-adjacent edges are not allowed to intersect.
            if (!loop->IsValid()) {
                return BAD_VALUE_STATUS;
            }

            // If the loop is more than one hemisphere, invert it.
            loop->Normalize();

            // Check the first loop must be the exterior ring and any others must be
            // interior rings or holes.
            if (loops.size() > 1 && !loops[0]->Contains(loop)) return BAD_VALUE_STATUS;
        }

        // Check if the given loops form a valid polygon.
        // 1. If a loop contains an edge AB, then no other loop may contain AB or BA.
        // 2. No loop covers more than half of the sphere.
        // 3. No two loops cross.
        if (!S2Polygon::IsValid(loops.vector())) return BAD_VALUE_STATUS;

        // Given all loops are valid / normalized and S2Polygon::IsValid() above returns true.
        // The polygon must be valid. See S2Polygon member function IsValid().

        // Transfer ownership of the loops and clears loop vector.
        out->Init(&loops.mutableVector());

        // Check if every loop of this polygon shares at most one vertex with
        // its parent loop.
        if (!out->IsNormalized()) return BAD_VALUE_STATUS;

        // S2Polygon contains more than one ring, which is allowed by S2, but not by GeoJSON.
        //
        // Loops are indexed according to a preorder traversal of the nesting hierarchy.
        // GetLastDescendant() returns the index of the last loop that is contained within
        // a given loop. We guarantee that the first loop is the exterior ring.
        if (out->GetLastDescendant(0) < out->num_loops() - 1) return BAD_VALUE_STATUS;

        // In GeoJSON, only one nesting is allowed.
        // The depth of a loop is set by polygon according to the nesting hierarchy of polygon,
        // so the exterior ring's depth is 0, a hole in it is 1, etc.
        for (int i = 0; i < out->num_loops(); i++) {
            if (out->loop(i)->depth() > 1) {
                return BAD_VALUE_STATUS;
            }
        }
        return Status::OK();
    }

    static Status parseBigSimplePolygonCoordinates(const BSONElement& elem,
                                                   BigSimplePolygon *out) {
        if (Array != elem.type()) return BAD_VALUE_STATUS;

        const vector<BSONElement>& coordinates = elem.Array();
        // Only one loop is allowed in a BigSimplePolygon
        if (coordinates.size() != 1)
            return BAD_VALUE_STATUS;

        vector<S2Point> exteriorVertices;
        Status status = Status::OK();
        status = newParseArrayOfCoodinates(coordinates.front(), &exteriorVertices);
        if (!status.isOK()) return status;

        status = newIsLoopClosed(exteriorVertices);
        if (!status.isOK()) return status;

        eraseDuplicatePoints(&exteriorVertices);

        // The last point is duplicated.  We drop it, since S2Loop expects no
        // duplicate points
        exteriorVertices.resize(exteriorVertices.size() - 1);

        // S2 Polygon loops must have 3 vertices
        if (exteriorVertices.size() < 3)
            return BAD_VALUE_STATUS;

        auto_ptr<S2Loop> loop(new S2Loop(exteriorVertices));
        if (!loop->IsValid())
            return BAD_VALUE_STATUS;

        out->Init(loop.release());
        return Status::OK();
    }

    // Parse "crs" field of BSON object.
    // "crs": {
    //   "type": "name",
    //   "properties": {
    //     "name": "urn:ogc:def:crs:OGC:1.3:CRS84"
    //    }
    // }
    static Status newParseGeoJSONCRS(const BSONObj &obj, CRS* crs) {
        BSONElement crsElt = obj["crs"];
        *crs = SPHERE;
        // "crs" field doesn't exist, return the default SPHERE
        if (crsElt.eoo()) {
            return Status::OK();
        }

        if (!crsElt.isABSONObj()) return BAD_VALUE_STATUS;
        BSONObj crsObj = crsElt.embeddedObject();

        // "type": "name"
        if (String != crsObj["type"].type() || "name" != crsObj["type"].String())
            return BAD_VALUE_STATUS;

        // "properties"
        BSONElement propertiesElt = crsObj["properties"];
        if (!propertiesElt.isABSONObj()) return BAD_VALUE_STATUS;
        BSONObj propertiesObj = propertiesElt.embeddedObject();
        if (String != propertiesObj["name"].type()) return BAD_VALUE_STATUS;
        const string& name = propertiesObj["name"].String();
        if (CRS_CRS84 == name || CRS_EPSG_4326 == name) {
            *crs = SPHERE;
        } else if (CRS_STRICT_WINDING == name) {
            *crs = STRICT_SPHERE;
        } else {
            return BAD_VALUE_STATUS;
        }
        return Status::OK();
    }

    // Parse "coordinates" field of GeoJSON LineString
    // e.g. "coordinates": [ [100.0, 0.0], [101.0, 1.0] ]
    // Or a line in "coordinates" field of GeoJSON MultiLineString
    static Status newParseGeoJSONLineCoordinates(const BSONElement& elem, S2Polyline* out) {
        vector<S2Point> vertices;
        Status status = newParseArrayOfCoodinates(elem, &vertices);
        if (!status.isOK()) return status;

        eraseDuplicatePoints(&vertices);
        if (vertices.size() < 2) return BAD_VALUE_STATUS;

        // XXX change to status
        if (!S2Polyline::IsValid(vertices)) return BAD_VALUE_STATUS;
        out->Init(vertices);
        return Status::OK();
    }

    // Parse legacy point or GeoJSON point, used by geo near.
    // Only stored legacy points allow additional fields.
    Status parsePoint(const BSONElement &elem, PointWithCRS *out, bool allowAddlFields) {
        if (!elem.isABSONObj()) return BAD_VALUE_STATUS;

        BSONObj obj = elem.Obj();
        // location: [1, 2] or location: {x: 1, y:2}
        if (Array == elem.type() || obj.firstElement().isNumber()) {
            // Legacy point
            return GeoParser::newParseLegacyPoint(elem, out, allowAddlFields);
        }

        // GeoJSON point. location: { type: "Point", coordinates: [1, 2] }
        return GeoParser::newParseGeoJSONPoint(obj, out);
    }

    /** exported **/
    Status GeoParser::parseStoredPoint(const BSONElement &elem, PointWithCRS *out) {
        return parsePoint(elem, out, true);
    }

    Status GeoParser::parseQueryPoint(const BSONElement &elem, PointWithCRS *out) {
        return parsePoint(elem, out, false);
    }

    Status GeoParser::newParseLegacyBox(const BSONObj& obj, BoxWithCRS *out) {
        Point ptA, ptB;
        Status status = Status::OK();

        BSONObjIterator coordIt(obj);
        status = newParseFlatPoint(coordIt.next(), &ptA);
        if (!status.isOK()) { return status; }
        status = newParseFlatPoint(coordIt.next(), &ptB);
        if (!status.isOK()) { return status; }
        // XXX: VERIFY AREA >= 0

        out->box.init(ptA, ptB);
        out->crs = FLAT;
        return status;
    }

    Status GeoParser::newParseLegacyPolygon(const BSONObj& obj, PolygonWithCRS *out) {
        BSONObjIterator coordIt(obj);
        vector<Point> points;
        while (coordIt.more()) {
            Point p;
            // A coordinate
            Status status = newParseFlatPoint(coordIt.next(), &p);
            if (!status.isOK()) return status;
            points.push_back(p);
        }
        if (points.size() < 3) return BAD_VALUE_STATUS;
        out->oldPolygon.init(points);
        out->crs = FLAT;
        return Status::OK();
    }

    // { "type": "Point", "coordinates": [100.0, 0.0] }
    Status GeoParser::newParseGeoJSONPoint(const BSONObj &obj,  PointWithCRS *out) {
        Status status = Status::OK();
        // "crs"
        status = newParseGeoJSONCRS(obj, &out->crs);
        if (!status.isOK()) return status;
        out->crs = FLAT;

        // "coordinates"
        status = newParseFlatPoint(obj[GEOJSON_COORDINATES], &out->oldPoint);
        if (!status.isOK()) return status;

        // Projection
        if (!ShapeProjection::supportsProject(*out, SPHERE))
            return BAD_VALUE_STATUS;
        ShapeProjection::projectInto(out, SPHERE);
        return Status::OK();
    }

    // { "type": "LineString", "coordinates": [ [100.0, 0.0], [101.0, 1.0] ] }
    Status GeoParser::newParseGeoJSONLine(const BSONObj& obj, LineWithCRS* out)  {
        Status status = Status::OK();
        // "crs"
        status = newParseGeoJSONCRS(obj, &out->crs);
        if (!status.isOK()) return status;

        // "coordinates"
        status = newParseGeoJSONLineCoordinates(obj[GEOJSON_COORDINATES], &out->line);
        if (!status.isOK()) return status;

        out->crs = SPHERE;
        return Status::OK();
    }

    Status GeoParser::newParseGeoJSONPolygon(const BSONObj &obj, PolygonWithCRS *out) {
        const BSONElement coordinates = obj[GEOJSON_COORDINATES];

        Status status = Status::OK();
        // "crs"
        status = newParseGeoJSONCRS(obj, &out->crs);
        if (!status.isOK()) return status;

        // "coordinates"
        if (out->crs == SPHERE) {
            out->s2Polygon.reset(new S2Polygon());
            status = parseGeoJSONPolygonCoordinates(coordinates, out->s2Polygon.get());
        }
        else if (out->crs == STRICT_SPHERE) {
            out->bigPolygon.reset(new BigSimplePolygon());
            status = parseBigSimplePolygonCoordinates(coordinates, out->bigPolygon.get());
        }
        return status;
    }

    Status GeoParser::parseMultiPoint(const BSONObj &obj, MultiPointWithCRS *out) {
        Status status = Status::OK();
        status = newParseGeoJSONCRS(obj, &out->crs);
        if (!status.isOK()) return status;

        out->points.clear();
        BSONElement coordElt = obj.getFieldDotted(GEOJSON_COORDINATES);
        status = newParseArrayOfCoodinates(coordElt, &out->points);
        if (!status.isOK()) return status;

        if (0 == out->points.size()) return BAD_VALUE_STATUS;
        out->cells.resize(out->points.size());
        for (size_t i = 0; i < out->points.size(); ++i) {
            out->cells[i] = S2Cell(out->points[i]);
        }
        out->crs = SPHERE;

        return Status::OK();
    }

    Status GeoParser::parseMultiLine(const BSONObj &obj, MultiLineWithCRS *out) {
        Status status = Status::OK();
        status = newParseGeoJSONCRS(obj, &out->crs);
        if (!status.isOK()) return status;

        BSONElement coordElt = obj.getFieldDotted(GEOJSON_COORDINATES);
        if (Array != coordElt.type()) return BAD_VALUE_STATUS;

        vector<S2Polyline*>& lines = out->lines.mutableVector();
        lines.clear();

        BSONObjIterator it(coordElt.Obj());

        // Iterate array
        while (it.more()) {
            lines.push_back(new S2Polyline());
            status = newParseGeoJSONLineCoordinates(it.next(), lines.back());
            if (!status.isOK()) return status;
        }
        if (0 == lines.size()) { return BAD_VALUE_STATUS; }
        out->crs = SPHERE;

        return Status::OK();
    }

    Status GeoParser::parseMultiPolygon(const BSONObj &obj, MultiPolygonWithCRS *out) {
        Status status = Status::OK();
        status = newParseGeoJSONCRS(obj, &out->crs);
        if (!status.isOK()) return status;

        BSONElement coordElt = obj.getFieldDotted(GEOJSON_COORDINATES);
        if (Array != coordElt.type()) return BAD_VALUE_STATUS;

        vector<S2Polygon*>& polygons = out->polygons.mutableVector();
        polygons.clear();

        BSONObjIterator it(coordElt.Obj());
        // Iterate array
        while (it.more()) {
            polygons.push_back(new S2Polygon());
            status = parseGeoJSONPolygonCoordinates(it.next(), polygons.back());
            if (!status.isOK()) return status;
        }
        if (0 == polygons.size()) { return BAD_VALUE_STATUS; }
        out->crs = SPHERE;

        return Status::OK();
    }

    Status GeoParser::newParseLegacyCenter(const BSONObj& obj, CapWithCRS *out) {
        BSONObjIterator objIt(obj);

        // Center
        BSONElement center = objIt.next();
        Status status = newParseFlatPoint(center, &out->circle.center);
        if (!status.isOK()) return status;

        // Radius
        BSONElement radius = objIt.next();
        if (!radius.isNumber() || radius.number() < 0) { return BAD_VALUE_STATUS; }

        // No more
        if (objIt.more()) return BAD_VALUE_STATUS;

        out->circle.radius = radius.number();
        out->crs = FLAT;
        return Status::OK();
    }

    Status GeoParser::newParseCenterSphere(const BSONObj& obj, CapWithCRS *out) {
        BSONObjIterator objIt(obj);

        // Center
        BSONElement center = objIt.next();
        Point p;
        // Check the object has and only has 2 numbers.
        Status status = newParseFlatPoint(center, &p);
        if (!status.isOK()) return status;
        if (!isValidLngLat(p.x, p.y)) { return BAD_VALUE_STATUS; }
        S2Point centerPoint = coordToPoint(p.x, p.y);

        // Radius
        BSONElement radiusElt = objIt.next();
        if (!radiusElt.isNumber() || radiusElt.number() < 0) { return BAD_VALUE_STATUS; }
        double radius = radiusElt.number();

        // No more elements
        if (objIt.more()) return BAD_VALUE_STATUS;

        out->cap = S2Cap::FromAxisAngle(centerPoint, S1Angle::Radians(radius));
        out->circle.radius = radius;
        out->circle.center = p;
        out->crs = SPHERE;
        return Status::OK();
    }

    //  { "type": "GeometryCollection",
    //    "geometries": [
    //      { "type": "Point",
    //        "coordinates": [100.0, 0.0]
    //      },
    //      { "type": "LineString",
    //        "coordinates": [ [101.0, 0.0], [102.0, 1.0] ]
    //      }
    //    ]
    //  }
    Status GeoParser::parseGeometryCollection(const BSONObj &obj, GeometryCollection *out) {
        BSONElement coordElt = obj.getFieldDotted(GEOJSON_GEOMETRIES);
        if (Array != coordElt.type()) { return BAD_VALUE_STATUS; }

        const vector<BSONElement>& geometries = coordElt.Array();
        if (0 == geometries.size()) { return BAD_VALUE_STATUS; }

        for (size_t i = 0; i < geometries.size(); ++i) {
            if (Object != geometries[i].type()) return BAD_VALUE_STATUS;

            const BSONObj& geoObj = geometries[i].Obj();
            GeoJSONType type = parseGeoJSONType(geoObj);

            if (GEOJSON_UNKNOWN == type || GEOJSON_GEOMETRY_COLLECTION == type) return BAD_VALUE_STATUS;

            Status status = Status::OK();
            if (GEOJSON_POINT == type) {
                out->points.resize(out->points.size() + 1);
                status = newParseGeoJSONPoint(geoObj, &out->points.back());
            } else if (GEOJSON_LINESTRING == type) {
                out->lines.mutableVector().push_back(new LineWithCRS());
                status = newParseGeoJSONLine(geoObj, out->lines.vector().back());
            } else if (GEOJSON_POLYGON == type) {
                out->polygons.mutableVector().push_back(new PolygonWithCRS());
                status = newParseGeoJSONPolygon(geoObj, out->polygons.vector().back());
            } else if (GEOJSON_MULTI_POINT == type) {
                out->multiPoints.mutableVector().push_back(new MultiPointWithCRS());
                status = parseMultiPoint(geoObj, out->multiPoints.mutableVector().back());
            } else if (GEOJSON_MULTI_LINESTRING == type) {
                out->multiLines.mutableVector().push_back(new MultiLineWithCRS());
                status = parseMultiLine(geoObj, out->multiLines.mutableVector().back());
            } else if (GEOJSON_MULTI_POLYGON == type) {
                out->multiPolygons.mutableVector().push_back(new MultiPolygonWithCRS());
                status = parseMultiPolygon(geoObj, out->multiPolygons.mutableVector().back());
            } else {
                // Should not reach here.
                invariant(false);
            }

            // Check parsing result.
            if (!status.isOK()) return status;
        }

        return Status::OK();
    }

    bool GeoParser::parsePointWithMaxDistance(const BSONObj& obj, PointWithCRS* out, double* maxOut) {
        BSONObjIterator it(obj);
        if (!it.more()) { return false; }

        BSONElement lng = it.next();
        if (!lng.isNumber()) { return false; }
        if (!it.more()) { return false; }

        BSONElement lat = it.next();
        if (!lat.isNumber()) { return false; }
        if (!it.more()) { return false; }

        BSONElement dist = it.next();
        if (!dist.isNumber()) { return false; }
        if (it.more()) { return false; }

        out->oldPoint.x = lng.number();
        out->oldPoint.y = lat.number();
        out->crs = FLAT;
        *maxOut = dist.number();
        return true;
    }

    GeoParser::GeoSpecifier GeoParser::parseGeoSpecifier(const BSONElement& type) {
        if (!type.isABSONObj()) { return GeoParser::UNKNOWN; }
        const char* fieldName = type.fieldName();
        if (mongoutils::str::equals(fieldName, "$box")) {
            return GeoParser::BOX;
        } else if (mongoutils::str::equals(fieldName, "$center")) {
            return GeoParser::CENTER;
        } else if (mongoutils::str::equals(fieldName, "$polygon")) {
            return GeoParser::POLYGON;
        } else if (mongoutils::str::equals(fieldName, "$centerSphere")) {
            return GeoParser::CENTER_SPHERE;
        } else if (mongoutils::str::equals(fieldName, "$geometry")) {
            return GeoParser::GEOMETRY;
        }
        return GeoParser::UNKNOWN;
    }

    GeoParser::GeoJSONType GeoParser::parseGeoJSONType(const BSONObj& obj) {
        BSONElement type = obj.getFieldDotted(GEOJSON_TYPE);
        if (String != type.type()) { return GeoParser::GEOJSON_UNKNOWN; }
        const string& typeString = type.String();
        if (GEOJSON_TYPE_POINT == typeString) {
            return GeoParser::GEOJSON_POINT;
        } else if (GEOJSON_TYPE_LINESTRING == typeString) {
            return GeoParser::GEOJSON_LINESTRING;
        } else if (GEOJSON_TYPE_POLYGON == typeString) {
            return GeoParser::GEOJSON_POLYGON;
        } else if (GEOJSON_TYPE_MULTI_POINT == typeString) {
            return GeoParser::GEOJSON_MULTI_POINT;
        } else if (GEOJSON_TYPE_MULTI_LINESTRING == typeString) {
            return GeoParser::GEOJSON_MULTI_LINESTRING;
        } else if (GEOJSON_TYPE_MULTI_POLYGON == typeString) {
            return GeoParser::GEOJSON_MULTI_POLYGON;
        } else if (GEOJSON_TYPE_GEOMETRY_COLLECTION == typeString) {
            return GeoParser::GEOJSON_GEOMETRY_COLLECTION;
        }
        return GeoParser::GEOJSON_UNKNOWN;
    }

}  // namespace mongo
