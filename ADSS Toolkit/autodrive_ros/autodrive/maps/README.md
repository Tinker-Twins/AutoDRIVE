Save map using `map_server` package (recommended):

```bash
$ cd <directory/where/to/save/the/map>
$ rosrun map_server map_saver -f <map_name>
```

Save map using `hector_geotiff` node:

```bash
$ rostopic pub syscommand std_msgs/String "savegeotiff"
```
