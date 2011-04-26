#!/bin/bash
neato -Tsvg -o metric_map.svg metric_map.dot
neato -Tsvg -o topo_map.svg topo_map.dot
convert metric_map.svg metric_map.png
convert topo_map.svg topo_map.png
