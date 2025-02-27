from pyproj import Proj, Transformer

wgs84 = Proj(proj="latlong", datum="WGS84")
utm = Proj(proj="utm", zone=32, datum="WGS84")

transformer_to_cartesian = Transformer.from_proj(wgs84, utm)
transformer_to_geodetic = Transformer.from_proj(utm, wgs84)

def geodetic_to_cartesian(lat, lon, alt):
    easting, northing = transformer_to_cartesian.transform(lat, lon)
    return easting, northing, alt

def cartesian_to_geodetic(easting, northing, alt):
    lat, lon = transformer_to_geodetic.transform(easting, northing)
    return lat, lon, alt

ara = geodetic_to_cartesian(0, 0, 0)
print(ara)
