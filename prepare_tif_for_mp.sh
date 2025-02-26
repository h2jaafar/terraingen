#!/bin/bash

# Ensure a GeoTIFF file is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <input.tif>"
    exit 1
fi

INPUT_TIF="$1"
OUTPUT_TIF="output_wgs84.tif"
RESAMPLED_TIF="aligned_wgs84_30m.tif"
TERRAIN_FOLDER="$(pwd)"

echo "🔍 Checking Projection of $INPUT_TIF..."
gdalinfo "$INPUT_TIF" | grep "Coordinate System"

# 🔍 Extract bounding box dynamically (in UTM)
echo "🔍 Extracting bounding box..."
BBOX=($(gdalinfo "$INPUT_TIF" | grep "Corner Coordinates" -A 5 | grep -E "Upper Left|Lower Right" | awk '{print $4, $5}' | tr -d '(),'))

XMIN=${BBOX[0]}
YMAX=${BBOX[1]}
XMAX=${BBOX[2]}
YMIN=${BBOX[3]}

# ✅ Convert bounding box to WGS84 using `gdaltransform`
read XMIN YMIN <<< $(echo "$XMIN $YMIN" | gdaltransform -s_srs EPSG:32621 -t_srs EPSG:4326 | awk '{print $1, $2}')
read XMAX YMAX <<< $(echo "$XMAX $YMAX" | gdaltransform -s_srs EPSG:32621 -t_srs EPSG:4326 | awk '{print $1, $2}')

echo "🌍 Bounding Box (WGS84): XMIN=$XMIN, YMIN=$YMIN, XMAX=$XMAX, YMAX=$YMAX"

echo "🌍 Reprojecting to WGS84 (EPSG:4326)..."
gdalwarp -t_srs EPSG:4326 -overwrite "$INPUT_TIF" "$OUTPUT_TIF"

echo "📏 Resampling to 30m grid..."
gdalwarp -te $XMIN $YMIN $XMAX $YMAX -ts 3601 3601 -r bilinear -overwrite "$OUTPUT_TIF" "$RESAMPLED_TIF"

# ✅ Correct `.HGT` filename format
LAT_PREFIX=$(echo "$YMIN" | awk '{printf "%02d", int($1)}')
LON_PREFIX=$(echo "$XMIN" | awk '{printf "%03d", int(-$1)}')  # Convert longitude to 3 digits, remove negative
HEMISPHERE_NS=$(echo "$YMIN" | awk '{print ($1 < 0 ? "S" : "N")}')
HEMISPHERE_EW=$(echo "$XMIN" | awk '{print ($1 < 0 ? "W" : "E")}')

RENAMED_TIF="${HEMISPHERE_NS}${LAT_PREFIX}${HEMISPHERE_EW}${LON_PREFIX}.tif"
mv "$RESAMPLED_TIF" "$RENAMED_TIF"

echo "✅ Terrain Generation Complete!"
echo "🔔 Reminder: Steps"
echo "1️⃣  Delete contents of ProgramData/MissionPlanner/srtm"
echo "2️⃣  Copy the new files to ProgramData/MissionPlanner/srtm"
echo "3️⃣  Turn off wifi and restart Mission Planner"
echo "4️⃣  Click Plan tab and go to region of interest, alt click and drag to select area"
echo "5️⃣  Right click and select Make Terrain DAT" 
echo "6️⃣  Copy the new DAT which is under Documents/Mission Planner/TerrainData to APM/terrain"
echo "🗑️  Cleaning up temporary files..."
rm "$OUTPUT_TIF"

echo "🚀 Done!"
