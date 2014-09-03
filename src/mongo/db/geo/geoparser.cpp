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

    static bool isGeoJSONPoint(const BSONObj& obj) {
        BSONElement type = obj.getFieldDotted(GEOJSON_TYPE);
        if (type.eoo() || (String != type.type())) { return false; }
        if (GEOJSON_TYPE_POINT != type.String()) { return false; }

        if (!GeoParser::crsIsOK(obj)) {
            warning() << "Invalid CRS: " << obj.toString() << endl;
            return false;
        }

        BSONElement coordElt = obj.getFieldDotted(GEOJSON_COORDINATES);
        if (coordElt.eoo() || (Array != coordElt.type())) { return false; }

        const vector<BSONElement>& coordinates = coordElt.Array();
        if (coordinates.size() != 2) { return false; }
        if (!coordinates[0].isNumber() || !coordinates[1].isNumber()) { return false; }
        // For now, we assume all GeoJSON must be within WGS84 - this may change
        double lat = coordinates[1].Number();
        double lng = coordinates[0].Number();
        return isValidLngLat(lng, lat);
    }

    static bool isLegacyPoint(const BSONObj &obj, bool allowAddlFields) {
        BSONObjIterator it(obj);
        if (!it.more()) { return false; }
        BSONElement x = it.next();
        if (!x.isNumber()) { return false; }
        if (!it.more()) { return false; }
        BSONElement y = it.next();
        if (!y.isNumber()) { return false; }
        if (it.more() && !allowAddlFields) { return false; }
        return true;
    }

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

    static bool isArrayOfCoordinates(const vector<BSONElement>& coordinateArray) {
        for (size_t i = 0; i < coordinateArray.size(); ++i) {
            // Each coordinate should be an array
            if (Array != coordinateArray[i].type()) { return false; }
            // ...of two
            const vector<BSONElement> &thisCoord = coordinateArray[i].Array();
            if (2 != thisCoord.size()) { return false; }
            // ...numbers.
            for (size_t j = 0; j < thisCoord.size(); ++j) {
                if (!thisCoord[j].isNumber()) { return false; }
            }
            // ...where the longitude, latitude is valid
            double lat = thisCoord[1].Number();
            double lng = thisCoord[0].Number();
            if (!isValidLngLat(lng, lat)) { return false; }
        }
        return true;
    }

    static Status newIsLoopClosed(const vector<S2Point>& loop) {
        if (loop.empty() || loop[0] != loop[loop.size() - 1]) return BAD_VALUE_STATUS;
        return Status::OK();
    }

    static bool parsePoints(const vector<BSONElement>& coordElt, vector<S2Point>* out) {
        for (size_t i = 0; i < coordElt.size(); ++i) {
            const vector<BSONElement>& pointElt = coordElt[i].Array();
            if (pointElt.empty()) { continue; }
            if (!isValidLngLat(pointElt[0].Number(), pointElt[1].Number())) {
                return false;
            }
            out->push_back(coordToPoint(pointElt[0].Number(), pointElt[1].Number()));
        }

        return true;
    }

    static bool isValidLineString(const vector<BSONElement>& coordinateArray) {
        if (coordinateArray.size() < 2) { return false; }
        if (!isArrayOfCoordinates(coordinateArray)) { return false; }
        vector<S2Point> vertices;
        if (!parsePoints(coordinateArray, &vertices)) { return false; }
        eraseDuplicatePoints(&vertices);
        return S2Polyline::IsValid(vertices);
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

    static bool parseLegacyPoint(const BSONObj &obj, Point *out) {
        BSONObjIterator it(obj);
        BSONElement x = it.next();
        BSONElement y = it.next();
        out->x = x.number();
        out->y = y.number();
        return true;
    }

    // Coordinates looks like [[0,0],[5,0],[5,5],[0,5],[0,0]]
    static bool isLoopClosed(const vector<BSONElement>& coordinates) {
        double x1, y1, x2, y2;
        x1 = coordinates[0].Array()[0].Number();
        y1 = coordinates[0].Array()[1].Number();
        x2 = coordinates[coordinates.size() - 1].Array()[0].Number();
        y2 = coordinates[coordinates.size() - 1].Array()[1].Number();
        return (fabs(x1 - x2) < 1e-6) && fabs(y1 - y2) < 1e-6;
    }

    static bool isGeoJSONPolygonCoordinates(const vector<BSONElement>& coordinates) {
        // Must be at least one element, the outer shell
        if (coordinates.empty()) { return false; }
        // Verify that the shell is a bunch'a coordinates.
        for (size_t i = 0; i < coordinates.size(); ++i) {
            if (Array != coordinates[i].type()) { return false; }
            const vector<BSONElement>& thisLoop = coordinates[i].Array();
            // A triangle is the simplest 2d shape, and we repeat a vertex, so, 4.
            if (thisLoop.size() < 4) { return false; }
            if (!isArrayOfCoordinates(thisLoop)) { return false; }
            if (!isLoopClosed(thisLoop)) { return false; }
        }
        return true;
    }

    static bool isGeoJSONPolygon(const BSONObj& obj) {
        BSONElement type = obj.getFieldDotted(GEOJSON_TYPE);
        if (type.eoo() || (String != type.type())) { return false; }
        if (GEOJSON_TYPE_POLYGON != type.String()) { return false; }

        if (!GeoParser::crsIsOK(obj)) {
            warning() << "Invalid CRS: " << obj.toString() << endl;
            return false;
        }

        BSONElement coordElt = obj.getFieldDotted(GEOJSON_COORDINATES);
        if (coordElt.eoo() || (Array != coordElt.type())) { return false; }

        return isGeoJSONPolygonCoordinates(coordElt.Array());
    }

    static bool isLegacyPolygon(const BSONObj &obj) {
        BSONObjIterator typeIt(obj);
        BSONElement type = typeIt.next();
        if (!type.isABSONObj()) { return false; }
        if (!mongoutils::str::equals(type.fieldName(), "$polygon")) { return false; }
        BSONObjIterator coordIt(type.embeddedObject());
        int vertices = 0;
        while (coordIt.more()) {
            BSONElement coord = coordIt.next();
            if (!coord.isABSONObj()) { return false; }
            if (!isLegacyPoint(coord.Obj(), false)) { return false; }
            ++vertices;
        }
        if (vertices < 3) { return false; }
        return true;
    }

    static bool isLegacyCenter(const BSONObj &obj) {
        BSONObjIterator typeIt(obj);
        BSONElement type = typeIt.next();
        if (!type.isABSONObj()) { return false; }
        bool isCenter = mongoutils::str::equals(type.fieldName(), "$center");
        if (!isCenter) { return false; }
        BSONObjIterator objIt(type.embeddedObject());
        BSONElement center = objIt.next();
        if (!center.isABSONObj()) { return false; }
        if (!isLegacyPoint(center.Obj(), false)) { return false; }
        if (!objIt.more()) { return false; }
        BSONElement radius = objIt.next();
        if (!radius.isNumber()) { return false; }
        return true;
    }

    static bool isLegacyCenterSphere(const BSONObj &obj) {
        BSONObjIterator typeIt(obj);
        BSONElement type = typeIt.next();
        if (!type.isABSONObj()) { return false; }
        bool isCenterSphere = mongoutils::str::equals(type.fieldName(), "$centerSphere");
        if (!isCenterSphere) { return false; }
        BSONObjIterator objIt(type.embeddedObject());
        BSONElement center = objIt.next();
        if (!center.isABSONObj()) { return false; }
        if (!isLegacyPoint(center.Obj(), false)) { return false; }
        // Check to make sure the points are valid lng/lat.
        BSONObjIterator coordIt(center.Obj());
        BSONElement lng = coordIt.next();
        BSONElement lat = coordIt.next();
        if (!isValidLngLat(lng.Number(), lat.Number())) { return false; }
        if (!objIt.more()) { return false; }
        BSONElement radius = objIt.next();
        if (!radius.isNumber()) { return false; }
        return true;
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

    /** exported **/

    bool GeoParser::isPoint(const BSONObj &obj) {
        return isLegacyPoint(obj, false) || isGeoJSONPoint(obj);
    }

    bool GeoParser::isIndexablePoint(const BSONObj &obj) {
        return isLegacyPoint(obj, true) || isGeoJSONPoint(obj);
    }

    bool GeoParser::parsePoint(const BSONObj &obj, PointWithCRS *out) {
        if (isLegacyPoint(obj, true)) {
            parseLegacyPoint(obj, &out->oldPoint);
            out->crs = FLAT;
        } else if (isGeoJSONPoint(obj)) {
            const vector<BSONElement>& coords = obj.getFieldDotted(GEOJSON_COORDINATES).Array();
            out->oldPoint.x = coords[0].Number();
            out->oldPoint.y = coords[1].Number();
            out->crs = FLAT;
            if (!ShapeProjection::supportsProject(*out, SPHERE))
                return false;
            ShapeProjection::projectInto(out, SPHERE);
        }
        return true;
    }

    bool GeoParser::isLine(const BSONObj& obj) {
        BSONElement type = obj.getFieldDotted(GEOJSON_TYPE);
        if (type.eoo() || (String != type.type())) { return false; }
        if (GEOJSON_TYPE_LINESTRING != type.String()) { return false; }

        if (!crsIsOK(obj)) {
            warning() << "Invalid CRS: " << obj.toString() << endl;
            return false;
        }

        BSONElement coordElt = obj.getFieldDotted(GEOJSON_COORDINATES);
        if (coordElt.eoo() || (Array != coordElt.type())) { return false; }

        return isValidLineString(coordElt.Array());
    }

    bool GeoParser::parseLine(const BSONObj& obj, LineWithCRS* out) {
        vector<S2Point> vertices;
        if (!parsePoints(obj.getFieldDotted(GEOJSON_COORDINATES).Array(), &vertices)) {
            return false;
        }
        eraseDuplicatePoints(&vertices);
        out->line.Init(vertices);
        out->crs = SPHERE;
        return true;
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

    bool GeoParser::isBox(const BSONObj &obj) {
        BSONObjIterator typeIt(obj);
        BSONElement type = typeIt.next();
        if (!type.isABSONObj()) { return false; }
        if (!mongoutils::str::equals(type.fieldName(), "$box")) { return false; }
        BSONObjIterator coordIt(type.embeddedObject());
        BSONElement minE = coordIt.next();
        if (!minE.isABSONObj()) { return false; }
        if (!isLegacyPoint(minE.Obj(), false)) { return false; }
        if (!coordIt.more()) { return false; }
        BSONElement maxE = coordIt.next();
        if (!maxE.isABSONObj()) { return false; }
        if (!isLegacyPoint(maxE.Obj(), false)) { return false; }
        // XXX: VERIFY AREA >= 0
        return true;
    }

    bool GeoParser::parseBox(const BSONObj &obj, BoxWithCRS *out) {
        BSONObjIterator typeIt(obj);
        BSONElement type = typeIt.next();
        return GeoParser::newParseLegacyBox(type.Obj(), out).isOK();
    }

    bool GeoParser::parsePolygon(const BSONObj &obj, PolygonWithCRS *out) {
        if (isGeoJSONPolygon(obj)) {
            return newParseGeoJSONPolygon(obj, out).isOK();
        } else {
            BSONObjIterator typeIt(obj);
            BSONElement type = typeIt.next();
            return newParseLegacyPolygon(type.Obj(), out).isOK();
        }
    }

    bool GeoParser::isMultiPoint(const BSONObj &obj) {
        BSONElement type = obj.getFieldDotted(GEOJSON_TYPE);
        if (type.eoo() || (String != type.type())) { return false; }
        if (GEOJSON_TYPE_MULTI_POINT != type.String()) { return false; }

        if (!crsIsOK(obj)) {
            warning() << "Invalid CRS: " << obj.toString() << endl;
            return false;
        }

        BSONElement coordElt = obj.getFieldDotted(GEOJSON_COORDINATES);
        if (coordElt.eoo() || (Array != coordElt.type())) { return false; }

        const vector<BSONElement>& coordinates = coordElt.Array();
        if (0 == coordinates.size()) { return false; }
        return isArrayOfCoordinates(coordinates);
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

    bool GeoParser::isMultiLine(const BSONObj &obj) {
        BSONElement type = obj.getFieldDotted(GEOJSON_TYPE);
        if (type.eoo() || (String != type.type())) { return false; }
        if (GEOJSON_TYPE_MULTI_LINESTRING != type.String()) { return false; }

        if (!crsIsOK(obj)) {
            warning() << "Invalid CRS: " << obj.toString() << endl;
            return false;
        }

        BSONElement coordElt = obj.getFieldDotted(GEOJSON_COORDINATES);
        if (coordElt.eoo() || (Array != coordElt.type())) { return false; }

        const vector<BSONElement>& coordinates = coordElt.Array();
        if (0 == coordinates.size()) { return false; }

        for (size_t i = 0; i < coordinates.size(); ++i) {
            if (coordinates[i].eoo() || (Array != coordinates[i].type())) { return false; }
            if (!isValidLineString(coordinates[i].Array())) { return false; }
        }

        return true;
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

    bool GeoParser::isMultiPolygon(const BSONObj &obj) {
        BSONElement type = obj.getFieldDotted(GEOJSON_TYPE);
        if (type.eoo() || (String != type.type())) { return false; }
        if (GEOJSON_TYPE_MULTI_POLYGON != type.String()) { return false; }

        if (!crsIsOK(obj)) {
            warning() << "Invalid CRS: " << obj.toString() << endl;
            return false;
        }

        BSONElement coordElt = obj.getFieldDotted(GEOJSON_COORDINATES);
        if (coordElt.eoo() || (Array != coordElt.type())) { return false; }

        const vector<BSONElement>& coordinates = coordElt.Array();
        if (0 == coordinates.size()) { return false; }
        for (size_t i = 0; i < coordinates.size(); ++i) {
            if (coordinates[i].eoo() || (Array != coordinates[i].type())) { return false; }
            if (!isGeoJSONPolygonCoordinates(coordinates[i].Array())) { return false; }
        }

        return true;
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

    bool GeoParser::isGeometryCollection(const BSONObj &obj) {
        BSONElement type = obj.getFieldDotted(GEOJSON_TYPE);
        if (type.eoo() || (String != type.type())) { return false; }
        if (GEOJSON_TYPE_GEOMETRY_COLLECTION != type.String()) { return false; }

        BSONElement coordElt = obj.getFieldDotted(GEOJSON_GEOMETRIES);
        if (coordElt.eoo() || (Array != coordElt.type())) { return false; }

        const vector<BSONElement>& coordinates = coordElt.Array();
        if (0 == coordinates.size()) { return false; }

        for (size_t i = 0; i < coordinates.size(); ++i) {
            if (coordinates[i].eoo() || (Object != coordinates[i].type())) { return false; }
            BSONObj obj = coordinates[i].Obj();
            if (!isGeoJSONPoint(obj) && !isLine(obj) && !isGeoJSONPolygon(obj)
                && !isMultiPoint(obj) && !isMultiPolygon(obj) && !isMultiLine(obj)) {
                return false;
            }
        }

        return true;
    }

    bool GeoParser::isPolygon(const BSONObj &obj) {
        return isGeoJSONPolygon(obj) || isLegacyPolygon(obj);
    }

    bool GeoParser::crsIsOK(const BSONObj &obj) {
        if (!obj.hasField("crs")) { return true; }

        if (!obj["crs"].isABSONObj()) { return false; }

        BSONObj crsObj = obj["crs"].embeddedObject();
        if (!crsObj.hasField("type")) { return false; }
        if (String != crsObj["type"].type()) { return false; }
        if ("name" != crsObj["type"].String()) { return false; }
        if (!crsObj.hasField("properties")) { return false; }
        if (!crsObj["properties"].isABSONObj()) { return false; }

        BSONObj propertiesObj = crsObj["properties"].embeddedObject();
        if (!propertiesObj.hasField("name")) { return false; }
        if (String != propertiesObj["name"].type()) { return false; }
        const string& name = propertiesObj["name"].String();

        // see http://portal.opengeospatial.org/files/?artifact_id=24045
        // and http://spatialreference.org/ref/epsg/4326/
        // and http://www.geojson.org/geojson-spec.html#named-crs
        return ("urn:ogc:def:crs:OGC:1.3:CRS84" == name) || ("EPSG:4326" == name) ||
               ("urn:mongodb:strictwindingcrs:EPSG:4326" == name);
    }

    bool GeoParser::parseGeoJSONCRS(const BSONObj& obj, CRS* crs) {

        dassert(crsIsOK(obj));

        *crs = SPHERE;

        if (!obj["crs"].eoo()) {
            const string name = obj["crs"].Obj()["properties"].Obj()["name"].String();

            if (name == "urn:mongodb:strictwindingcrs:EPSG:4326")
                *crs = STRICT_SPHERE;
            else
                *crs = SPHERE;
        }

        return true;
    }

    bool GeoParser::isCap(const BSONObj &obj) {
        return isLegacyCenter(obj) || isLegacyCenterSphere(obj);
    }

    Status GeoParser::newParseLegacyCenter(const BSONObj& obj, CapWithCRS *out) {
        BSONObjIterator objIt(obj);

        // Center
        BSONElement center = objIt.next();
        Status status = newParseFlatPoint(center, &out->circle.center);
        if (!status.isOK()) return status;

        // Redius
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

    bool GeoParser::parseCap(const BSONObj& obj, CapWithCRS *out) {
        if (isLegacyCenter(obj)) {
            BSONObjIterator typeIt(obj);
            BSONElement type = typeIt.next();
            return newParseLegacyCenter(type.Obj(), out).isOK();
        } else {
            verify(isLegacyCenterSphere(obj));
            BSONObjIterator typeIt(obj);
            BSONElement type = typeIt.next();
            return newParseCenterSphere(type.Obj(), out).isOK();
        }
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
