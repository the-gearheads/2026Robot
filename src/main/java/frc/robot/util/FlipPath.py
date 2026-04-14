import json

import copy

import re




def reflect_y(y, axis=4.034536):

    return 2 * axis - y




def reflect_point(pt, axis=4.034536):

    if pt is None:

        return None

    return {"x": pt["x"], "y": reflect_y(pt["y"], axis)}




def flip_lr_name(name):

    if name is None:

        return None

    def replacer(m):

        word = m.group(0)

        if word == "Left":   return "Right"

        if word == "Right":  return "Left"

        if word == "left":   return "right"

        if word == "right":  return "left"

        if word == "LEFT":   return "RIGHT"

        if word == "RIGHT":  return "LEFT"

        return word

    return re.sub(r'\b(Left|Right|left|right|LEFT|RIGHT)\b', replacer, name)




with open("C:\\Users\\1189-Driver\\Documents\\FRC\\2026Robot\\src\\main\\deploy\\pathplanner\\paths\\RC1.path", "r") as f:

    data = json.load(f)




result = copy.deepcopy(data)




for wp in result["waypoints"]:

    wp["anchor"] = reflect_point(wp["anchor"])

    wp["prevControl"] = reflect_point(wp["prevControl"])

    wp["nextControl"] = reflect_point(wp["nextControl"])

    wp["linkedName"] = flip_lr_name(wp["linkedName"])




for rt in result["rotationTargets"]:

    rt["rotationDegrees"] = -rt["rotationDegrees"]




result["goalEndState"]["rotation"] = -result["goalEndState"]["rotation"]

result["idealStartingState"]["rotation"] = -result["idealStartingState"]["rotation"]




with open("C:\\Users\\1189-Driver\\Documents\\FRC\\2026Robot\\src\\main\\deploy\\pathplanner\\paths\\LC1.path", "w") as f:

    json.dump(result, f, indent=2)




print("Done! Saved to output.path")