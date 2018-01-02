#!/usr/bin/python
# -*- coding=utf-8 -*-

# author : dunwenqiang@gmail.com
# website: http://www.jb51.net/article/73450.htm
# date   : 2017-12-27
# version: 0.1

import json

# Read JSON data
with open('task_config.json', 'r') as config_file:
    task_property = json.load(config_file)
    task_name = task_property["task_name"]["city_code"]
    task_name += "-" + task_property["task_name"]["day_night"]
    task_name += "-" + task_property["task_name"]["task_number"] + task_property["task_name"]["car_number"]
    print task_name
