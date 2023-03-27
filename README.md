# Description

This package converts .pcd maps from local UTM coordinates to MGRS coordinates.
You need to provide the UTM maps origin coordinates (Northing, Easting, ElipsoidHeight).
It also uses elipsoid height and converts it to orthometric height.
# Usage

Launch the pc_utm_to_mgrs_converter.launch.xml file.
```bash
ros2 launch pc_utm_to_mgrs_converter pc_utm_to_mgrs_converter.launch.xml
```
# Parameters
| Name                | Type   | Description                            |
|---------------------|--------|----------------------------------------|
| `Northing`          | double | Northing of origin point               |
| `Easting`           | double | Easting of origin point                |
| `ElipsoidHeight`    | double | ElipsoidHeight of origin point         |
| `input_file_path`   | string | Your utm map path                      |
| `output_file_path`  | string | Directory where mgrs map will be saved |
