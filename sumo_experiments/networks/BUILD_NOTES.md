# Shahbagh SUMO network — build record (reproducibility note)
#
# Source: OpenStreetMap, bbox lon 90.386–90.404 / lat 23.730–23.746
#         (Shahbagh, Dhaka), fetched 2026-09-14 via the OSM main API
#         (api.openstreetmap.org/api/0.6/map). Raw extract kept as
#         `shahbagh.osm` (6.2 MB, 24,206 nodes / 3,720 ways).
#
# Build (SUMO 1.27.1, eclipse-sumo wheel):
#   netconvert --osm-files shahbagh.osm \
#     --output-file shahbagh.net.xml \
#     --geometry.remove --roundabouts.guess --ramps.guess \
#     --junctions.join --tls.guess-signals --tls.discard-simple \
#     --no-warnings
#
# Result: 1,528 real edges, 737 nodes, 94.6% in the largest weakly
# connected component, 4 signalized junctions. Demand generation
# (scripts/make_demand.py) restricts origins/destinations to the
# largest connected component so every trip is routable.
