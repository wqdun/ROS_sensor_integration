#!/usr/bin/python
# -*- coding=utf-8 -*-

# author : wklken@yeah.net
# website: http://python.jobbole.com/83593/
# date   : 2012-05-25
# version: 0.1
import sys
from xml.etree.ElementTree import ElementTree, Element

def read_xml(in_path):
    '''读取并解析xml文件
       in_path: xml路径
       return : ElementTree'''
    tree = ElementTree()
    tree.parse(in_path)
    return tree

def write_xml(tree, out_path):
    '''将xml文件写出
       tree    : xml树
       out_path: 写出路径'''
    tree.write(out_path, encoding="utf-8", xml_declaration = True)

def if_match(node, kv_map):
    '''判断某个节点是否包含所有传入参数属性
       node  : 节点
       kv_map: 属性及属性值组成的map'''
    for key in kv_map:
        if node.get(key) != kv_map.get(key):
            return False
    return True

#---------------search -----
def find_nodes(tree, path):
    '''查找某个路径匹配的所有节点
       tree: xml树
       path: 节点路径'''
    return tree.findall(path)

def get_node_by_keyvalue(nodelist, kv_map):
    '''根据属性及属性值定位符合的节点，返回节点
       nodelist: 节点列表
       kv_map  : 匹配属性及属性值map'''
    result_nodes = []
    for node in nodelist:
        if if_match(node, kv_map):
            result_nodes.append(node)
    return result_nodes

#---------------change -----
def change_node_properties(nodelist, kv_map, is_delete = False):
    '''修改/增加/删除 节点的属性及属性值
       nodelist: 节点列表
       kv_map  : 属性及属性值map'''
    for node in nodelist:
        for key in kv_map:
            if is_delete:
                if key in node.attrib:
                    del node.attrib[key]
            else:
                node.set(key, kv_map.get(key))

def change_node_text(nodelist, text, is_add = False, is_delete = False):
    '''改变/增加/删除一个节点的文本
       nodelist: 节点列表
       text    : 更新后的文本'''
    for node in nodelist:
        if is_add:
            node.text += text
        elif is_delete:
            node.text = ""
        else:
            node.text = text

def create_node(tag, property_map, content = ""):
    '''新造一个节点
       tag         : 节点标签
       property_map: 属性及属性值map
       content     : 节点闭合标签里的文本内容
       return      : 新节点'''
    element = Element(tag, property_map)
    element.text = content
    return element

def add_child_node(nodelist, element):
    '''给一个节点添加子节点
       nodelist: 节点列表
       element : 子节点'''
    for node in nodelist:
        node.append(element)

def del_node_by_tagkeyvalue(nodelist, tag, kv_map):
    '''通过属性及属性值定位一个节点，并删除之
       nodelist: 父节点列表
       tag     : 子节点标签
       kv_map  : 属性及属性值列表'''
    for parent_node in nodelist:
        children = parent_node.getchildren()
        for child in children:
            if child.tag == tag and if_match(child, kv_map):
                print tag, kv_map
                parent_node.remove(child)

if __name__ == "__main__":
    record_path = sys.argv[1]
    velodyne_launch = sys.argv[2]
    path_lidar = record_path + "/Lidar/"

    # 1 读取xml文件
    tree_cld = read_xml(velodyne_launch);
    # 2 属性修改
    # 2.1 找到父节点
    nodes_cld = find_nodes(tree_cld, "node")
    # 2.2 通过属性准确定位子节点
    # add child node
    result_node_cld = get_node_by_keyvalue(nodes_cld, {"name": "$(arg manager)_driver"})
    element = create_node("param", {"name": "record_path", "value": path_lidar}, "")
    add_child_node(result_node_cld, element)
    # 3 输出到结果文件
    new_velodyne_launch = velodyne_launch[:-6] + "xml"
    write_xml(tree_cld, new_velodyne_launch)
