/**
*    Copyright (C) 2013 10gen Inc.
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

#pragma once

#include "mongo/db/geo/shapes.h"
#include "mongo/db/jsobj.h"

namespace mongo {

    // This class parses geographic data.
    // It parses a subset of GeoJSON and creates S2 shapes from it.
    // See http://geojson.org/geojson-spec.html for the spec.
    //
    // This class also parses the ad-hoc geo formats that MongoDB introduced.
    //
    // parse* methods may do some more validation than the is* methods; they return false if they
    // encounter invalid geometry and true if the geometry is parsed successfully.
    class GeoParser {
    public:

        enum GeoSpecifier {
            BOX,
            CENTER,
            POLYGON,
            CENTER_SPHERE,
            GEOMETRY, // GeoJSON geometry
            UNKNOWN
        };

        enum GeoJSONType {
            GEOJSON_POINT,
            GEOJSON_LINESTRING,
            GEOJSON_POLYGON,
            GEOJSON_MULTI_POINT,
            GEOJSON_MULTI_LINESTRING,
            GEOJSON_MULTI_POLYGON,
            GEOJSON_GEOMETRY_COLLECTION,
            GEOJSON_UNKNOWN
        };

        static GeoSpecifier parseGeoSpecifier(const BSONElement& elem);
        static GeoJSONType parseGeoJSONType(const BSONObj& obj);

        static Status newParseLegacyPoint(const BSONElement &elem, Point *out, bool allowAddlFields = false);
        // Parse the BSON object after $box, $center, etc.
        static Status newParseLegacyBox(const BSONObj& obj, BoxWithCRS *out);
        static Status newParseLegacyCenter(const BSONObj& obj, CapWithCRS *out);
        static Status newParseLegacyPolygon(const BSONObj& obj, PolygonWithCRS *out);
        static Status newParseCenterSphere(const BSONObj& obj, CapWithCRS *out);
        static Status newParseGeoJSONPolygon(const BSONObj &obj, PolygonWithCRS *out);
        static Status newParseGeoJSONPoint(const BSONObj &obj,  PointWithCRS *out);
        static Status newParseGeoJSONLine(const BSONObj& obj, LineWithCRS* out);

        // XXX: Remove
        static bool isPoint(const BSONObj &obj);
        // Legacy points can contain extra data as extra fields - these are valid to index
        static bool isIndexablePoint(const BSONObj& obj);
        static bool parsePoint(const BSONObj &obj, PointWithCRS *out);

        static bool isLine(const BSONObj &obj);
        static bool parseLine(const BSONObj &obj, LineWithCRS *out);

        static bool isBox(const BSONObj &obj);
        static bool parseBox(const BSONObj &obj, BoxWithCRS *out);

        static bool isPolygon(const BSONObj &obj);
        static bool parsePolygon(const BSONObj &obj, PolygonWithCRS *out);

        // AKA $center or $centerSphere
        static bool isCap(const BSONObj &obj);
        static bool parseCap(const BSONObj &obj, CapWithCRS *out);
        // XXX: Remove above

        static bool isMultiPoint(const BSONObj &obj);
        static Status parseMultiPoint(const BSONObj &obj, MultiPointWithCRS *out);

        static bool isMultiLine(const BSONObj &obj);
        static Status parseMultiLine(const BSONObj &obj, MultiLineWithCRS *out);

        static bool isMultiPolygon(const BSONObj &obj);
        static Status parseMultiPolygon(const BSONObj &obj, MultiPolygonWithCRS *out);

        static bool isGeometryCollection(const BSONObj &obj);
        static Status parseGeometryCollection(const BSONObj &obj, GeometryCollection *out);

        static bool parsePointWithMaxDistance(const BSONObj& obj, PointWithCRS* out, double* maxOut);

        // Return true if the CRS field is 1. missing, or 2. is well-formed and
        // has a datum we accept.  Otherwise, return false.
        // NOTE(hk): If this is ever used anywhere but internally, consider
        // returning states: missing, invalid, unknown, ok, etc. -- whatever
        // needed.
        static bool crsIsOK(const BSONObj& obj);
        static bool parseGeoJSONCRS(const BSONObj& obj, CRS* crs);
    };

}  // namespace mongo
