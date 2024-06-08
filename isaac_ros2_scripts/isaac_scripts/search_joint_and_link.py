from omni.isaac.dynamic_control import _dynamic_control

def search_joint_prim_path(target_dict:dict, path:str, target_name:str):
    if target_dict["A_joint"] == target_name:
        path = path + target_dict["A_link"] + "/" +target_name
        return path
    #path = path + target_dict["B_link"] + "/"
    for child in target_dict["B_node"]:
        ret = search_joint_prim_path(child, path, target_name)
        if not ret == None:
            return ret
    return None

def search_link_prim_path(target_dict:dict, path:str, target_name:str):
    if target_dict["B_link"] == target_name:
        path = path + target_name
        return path
    #if "A_link" in target_dict:
    #    path = path + target_dict["A_link"] + "/"
    for child in target_dict["B_node"]:
        ret = search_link_prim_path(child, path, target_name)
        if not ret == None:
            return ret
    return None
    
def get_link_prim_path_list(target_dict:dict, path:str):
    path_list = []
    if not target_dict["B_link"] == "":
        target_path = path + target_dict["B_link"]
        path_list.append(target_path)
    for child in target_dict["B_node"]:
        path_list = path_list + get_link_prim_path_list(child, path)
    return path_list
    
def find_prim_path_by_name(start_stage, prim_name):
    for prim in start_stage.GetAllChildren():
        if prim.GetName() == prim_name:
            return prim.GetPath().pathString
        else:
            ret = find_prim_path_by_name(prim, prim_name)
            if not ret == None:
                return ret
    
    return None

def find_articulation_root(start_stage):
    dc = _dynamic_control.acquire_dynamic_control_interface()

    for prim in start_stage.GetAllChildren():
        art = dc.get_articulation(prim.GetPath().pathString)
        if not art == _dynamic_control.INVALID_HANDLE:
            return (art, prim.GetPath().pathString)
        else:
            ret = find_articulation_root(prim)
            if not ret == None:
                return ret

    return None
