string project_name

# 0: initial value; 1: shutdown; 2: reboot; 3: cleanup server process; 4: fix projects data; 5: remove projects data; 6: display recorded layers
int8 system_cmd

# separator: ,
string cmd_arguments

NodeParams node_params

ProjectArr project_arr
