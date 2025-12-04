import bpy
import sys
import os
import tqdm
import math
from random import randint
import time
from mathutils import Vector
from scipy.spatial.transform import Rotation as R


scriptpath = "C:/Users/maxd5/Documents/GitHub/deformable-pov-display/scripts (python)/troposkein modeling/"
sys.path.append(os.path.abspath(scriptpath))
from troposkein import *
import constants
import random
import cv2


def compute_led_rgba(r, g, b):
    pixel = np.array([r,g,b], dtype=np.float32)
    # Ensure the RGB values are in the range 0-255
    new_pixel = pixel/255.0 #np.max(pixel)
    # Compute the normalized intensity
    intensity = np.sum(pixel) / (3 * 255.0)

    return [*new_pixel, intensity]


#initialize PixelMaterial node group
def pixelmaterial_node_group(mat, emission_strength):

	pixelmaterial = mat.node_tree
	#start with a clean node tree
	for node in pixelmaterial.nodes:
		pixelmaterial.nodes.remove(node)
	#initialize pixelmaterial nodes
	#node Material Output
	material_output = pixelmaterial.nodes.new("ShaderNodeOutputMaterial")
	material_output.name = "Material Output"
	material_output.is_active_output = True
	material_output.target = 'ALL'
	#Displacement
	material_output.inputs[2].default_value = (0.0, 0.0, 0.0)
	#Thickness
	material_output.inputs[3].default_value = 0.0
	
	#node Mix Shader
	mix_shader = pixelmaterial.nodes.new("ShaderNodeMixShader")
	mix_shader.name = "Mix Shader"
	
	#node Attribute
	attribute = pixelmaterial.nodes.new("ShaderNodeAttribute")
	attribute.name = "Attribute"
	attribute.attribute_name = "Col"
	attribute.attribute_type = 'GEOMETRY'
	
	#node Value
	value = pixelmaterial.nodes.new("ShaderNodeValue")
	value.name = "Value"
	
	value.outputs[0].default_value = 1.0
	#node Gamma
	gamma = pixelmaterial.nodes.new("ShaderNodeGamma")
	gamma.name = "Gamma"
	
	#node Emission
	emission = pixelmaterial.nodes.new("ShaderNodeEmission")
	emission.name = "Emission"
	#Strength
	emission.inputs[1].default_value = emission_strength
	
	#node Transparent BSDF
	transparent_bsdf = pixelmaterial.nodes.new("ShaderNodeBsdfTransparent")
	transparent_bsdf.name = "Transparent BSDF"
	#Color
	transparent_bsdf.inputs[0].default_value = (1.0, 1.0, 1.0, 1.0)
	#Weight
	transparent_bsdf.inputs[1].default_value = 0.0
	
	
	#Set locations
	material_output.location = (797.7401123046875, 338.58795166015625)
	mix_shader.location = (526.1776733398438, 286.0436096191406)
	attribute.location = (-239.61517333984375, 311.7586975097656)
	value.location = (-116.46099090576172, 506.8948059082031)
	gamma.location = (140.03204345703125, 464.41766357421875)
	emission.location = (330.0133361816406, 417.3265075683594)
	transparent_bsdf.location = (215.13148498535156, 176.32342529296875)
	
	#Set dimensions
	material_output.width, material_output.height = 140.0, 100.0
	mix_shader.width, mix_shader.height = 140.0, 100.0
	attribute.width, attribute.height = 140.0, 100.0
	value.width, value.height = 140.0, 100.0
	gamma.width, gamma.height = 140.0, 100.0
	emission.width, emission.height = 140.0, 100.0
	transparent_bsdf.width, transparent_bsdf.height = 140.0, 100.0
	
	#initialize pixelmaterial links
	#attribute.Color -> gamma.Color
	pixelmaterial.links.new(attribute.outputs[0], gamma.inputs[0])
	#value.Value -> gamma.Gamma
	pixelmaterial.links.new(value.outputs[0], gamma.inputs[1])
	#mix_shader.Shader -> material_output.Surface
	pixelmaterial.links.new(mix_shader.outputs[0], material_output.inputs[0])
	#attribute.Alpha -> mix_shader.Fac
	pixelmaterial.links.new(attribute.outputs[3], mix_shader.inputs[0])
	#transparent_bsdf.BSDF -> mix_shader.Shader
	pixelmaterial.links.new(transparent_bsdf.outputs[0], mix_shader.inputs[1])
	#emission.Emission -> mix_shader.Shader
	pixelmaterial.links.new(emission.outputs[0], mix_shader.inputs[2])
	#gamma.Color -> emission.Color
	pixelmaterial.links.new(gamma.outputs[0], emission.inputs[0])
	return pixelmaterial





#initialize BackgroundMaterial node group
def backgroundmaterial_node_group(background_material, alpha):

	backgroundmaterial = background_material.node_tree
	#start with a clean node tree
	for node in backgroundmaterial.nodes:
		backgroundmaterial.nodes.remove(node)
	#initialize backgroundmaterial nodes
	#node Material Output
	material_output = backgroundmaterial.nodes.new("ShaderNodeOutputMaterial")
	material_output.name = "Material Output"
	material_output.is_active_output = True
	material_output.target = 'ALL'
	#Displacement
	material_output.inputs[2].default_value = (0.0, 0.0, 0.0)
	#Thickness
	material_output.inputs[3].default_value = 0.0
	
	#node Mix Shader
	mix_shader = backgroundmaterial.nodes.new("ShaderNodeMixShader")
	mix_shader.name = "Mix Shader"
	#Fac
	mix_shader.inputs[0].default_value = alpha
	
	#node Emission
	emission = backgroundmaterial.nodes.new("ShaderNodeEmission")
	emission.name = "Emission"
	#Color
	emission.inputs[0].default_value = (0.0, 0.0, 0.0, 1.0)
	#Strength
	emission.inputs[1].default_value = 1.0
	#Weight
	emission.inputs[2].default_value = 0.0
	
	#node Transparent BSDF
	transparent_bsdf = backgroundmaterial.nodes.new("ShaderNodeBsdfTransparent")
	transparent_bsdf.name = "Transparent BSDF"
	#Color
	transparent_bsdf.inputs[0].default_value = (1.0, 1.0, 1.0, 1.0)
	#Weight
	transparent_bsdf.inputs[1].default_value = 0.0
	
	
	#Set locations
	material_output.location = (607.87548828125, 224.60592651367188)
	mix_shader.location = (309.46893310546875, 190.40988159179688)
	emission.location = (-54.508365631103516, 279.511962890625)
	transparent_bsdf.location = (16.64507293701172, 53.329742431640625)
	
	#Set dimensions
	material_output.width, material_output.height = 140.0, 100.0
	mix_shader.width, mix_shader.height = 140.0, 100.0
	emission.width, emission.height = 140.0, 100.0
	transparent_bsdf.width, transparent_bsdf.height = 140.0, 100.0
	
	#initialize backgroundmaterial links
	#mix_shader.Shader -> material_output.Surface
	backgroundmaterial.links.new(mix_shader.outputs[0], material_output.inputs[0])
	#emission.Emission -> mix_shader.Shader
	backgroundmaterial.links.new(emission.outputs[0], mix_shader.inputs[2])
	#transparent_bsdf.BSDF -> mix_shader.Shader
	backgroundmaterial.links.new(transparent_bsdf.outputs[0], mix_shader.inputs[1])
	return backgroundmaterial




#initialize pixel_nodes node group
def pixel_nodes_node_group(pixel_material, background_material):
    pixel_nodes = bpy.data.node_groups.new(type = 'GeometryNodeTree', name = "Pixel Nodes")

    #initialize pixel_nodes nodes
    #node Set Material
    set_material = pixel_nodes.nodes.new("GeometryNodeSetMaterial")
    set_material.name = "Set Material"
    #Selection
    set_material.inputs[1].default_value = True

    #node Mesh Circle
    mesh_circle = pixel_nodes.nodes.new("GeometryNodeMeshCircle")
    mesh_circle.name = "Mesh Circle"
    mesh_circle.fill_type = 'TRIANGLE_FAN'
    #Vertices
    mesh_circle.inputs[0].default_value = 16
    #Radius
    mesh_circle.inputs[1].default_value = 0.20000000298023224

    #node Group Input
    group_input = pixel_nodes.nodes.new("NodeGroupInput")
    group_input.name = "Group Input"
    #pixel_nodes inputs
    #input Geometry
    pixel_nodes.inputs.new('NodeSocketGeometry', "Geometry")
    pixel_nodes.inputs[0].attribute_domain = 'POINT'

    #input Color Input
    pixel_nodes.inputs.new('NodeSocketMaterial', "Color Input")
    pixel_nodes.inputs[1].attribute_domain = 'POINT'
    pixel_nodes.inputs[1].default_value = pixel_material

    #node Group Output
    group_output = pixel_nodes.nodes.new("NodeGroupOutput")
    group_output.name = "Group Output"
    group_output.is_active_output = True
    #pixel_nodes outputs
    #output Geometry
    pixel_nodes.outputs.new('NodeSocketGeometry', "Geometry")
    pixel_nodes.outputs[0].attribute_domain = 'POINT'



    #node Align Euler to Vector
    align_euler_to_vector = pixel_nodes.nodes.new("FunctionNodeAlignEulerToVector")
    align_euler_to_vector.name = "Align Euler to Vector"
    align_euler_to_vector.axis = 'Z'
    align_euler_to_vector.pivot_axis = 'AUTO'
    #Rotation
    align_euler_to_vector.inputs[0].default_value = (0.0, 0.0, 0.0)
    #Factor
    align_euler_to_vector.inputs[1].default_value = 1.0

    #node Viewer
    viewer = pixel_nodes.nodes.new("GeometryNodeViewer")
    viewer.name = "Viewer"
    viewer.data_type = 'FLOAT_VECTOR'
    viewer.domain = 'AUTO'
    #Value_001
    viewer.inputs[2].default_value = (0.0, 0.0, 0.0)
    #Value_002
    viewer.inputs[3].default_value = (0.0, 0.0, 0.0, 0.0)
    #Value_003
    viewer.inputs[4].default_value = 0
    #Value_004
    viewer.inputs[5].default_value = False

    #node Instance on Points
    instance_on_points = pixel_nodes.nodes.new("GeometryNodeInstanceOnPoints")
    instance_on_points.name = "Instance on Points"
    #Selection
    instance_on_points.inputs[1].default_value = True
    #Pick Instance
    instance_on_points.inputs[3].default_value = False
    #Instance Index
    instance_on_points.inputs[4].default_value = 0
    #Scale
    instance_on_points.inputs[6].default_value = (1.0, 1.0, 1.0)

    #node Normal
    normal = pixel_nodes.nodes.new("GeometryNodeInputNormal")
    normal.name = "Normal"

    #node Realize Instances
    realize_instances = pixel_nodes.nodes.new("GeometryNodeRealizeInstances")
    realize_instances.name = "Realize Instances"
    realize_instances.legacy_behavior = False

    #node Join Geometry
    join_geometry = pixel_nodes.nodes.new("GeometryNodeJoinGeometry")
    join_geometry.name = "Join Geometry"

    #node Vector Math
    vector_math = pixel_nodes.nodes.new("ShaderNodeVectorMath")
    vector_math.name = "Vector Math"
    vector_math.operation = 'SCALE'
    #Vector_001
    vector_math.inputs[1].default_value = (0.0, 0.0, 0.0)
    #Vector_002
    vector_math.inputs[2].default_value = (0.0, 0.0, 0.0)
    #Scale
    vector_math.inputs[3].default_value = 0.02

    #node Set Material.001
    set_material_001 = pixel_nodes.nodes.new("GeometryNodeSetMaterial")
    set_material_001.name = "Set Material.001"
    #Selection
    set_material_001.inputs[1].default_value = True
    set_material_001.inputs[2].default_value = background_material

    #node Set Position
    set_position = pixel_nodes.nodes.new("GeometryNodeSetPosition")
    set_position.name = "Set Position"
    #Selection
    set_position.inputs[1].default_value = True
    #Position
    set_position.inputs[2].default_value = (0.0, 0.0, 0.0)



    #Set locations
    set_material.location = (-125.36164855957031, 39.40494155883789)
    mesh_circle.location = (-298.47296142578125, 134.21710205078125)
    group_input.location = (-643.4822998046875, 149.14231872558594)
    group_output.location = (1244.046875, 89.57275390625)
    align_euler_to_vector.location = (97.6287841796875, 10.001731872558594)
    viewer.location = (1170.795166015625, -86.60029602050781)
    instance_on_points.location = (263.7725830078125, 211.6618194580078)
    normal.location = (-245.72879028320312, -182.4083709716797)
    realize_instances.location = (505.724609375, 85.07298278808594)
    join_geometry.location = (987.618896484375, 127.21574401855469)
    vector_math.location = (336.5150451660156, -198.1876678466797)
    set_material_001.location = (-57.58793640136719, 338.8460388183594)
    set_position.location = (738.0050048828125, 18.911346435546875)

    #Set dimensions
    set_material.width, set_material.height = 140.0, 100.0
    mesh_circle.width, mesh_circle.height = 140.0, 100.0
    group_input.width, group_input.height = 140.0, 100.0
    group_output.width, group_output.height = 140.0, 100.0
    align_euler_to_vector.width, align_euler_to_vector.height = 140.0, 100.0
    viewer.width, viewer.height = 140.0, 100.0
    instance_on_points.width, instance_on_points.height = 140.0, 100.0
    normal.width, normal.height = 140.0, 100.0
    realize_instances.width, realize_instances.height = 140.0, 100.0
    join_geometry.width, join_geometry.height = 140.0, 100.0
    vector_math.width, vector_math.height = 140.0, 100.0
    set_material_001.width, set_material_001.height = 140.0, 100.0
    set_position.width, set_position.height = 140.0, 100.0

    #initialize pixel_nodes links
    #mesh_circle.Mesh -> set_material.Geometry
    pixel_nodes.links.new(mesh_circle.outputs[0], set_material.inputs[0])
    #set_material.Geometry -> instance_on_points.Instance
    pixel_nodes.links.new(set_material.outputs[0], instance_on_points.inputs[2])
    #align_euler_to_vector.Rotation -> instance_on_points.Rotation
    pixel_nodes.links.new(align_euler_to_vector.outputs[0], instance_on_points.inputs[5])
    #normal.Normal -> align_euler_to_vector.Vector
    pixel_nodes.links.new(normal.outputs[0], align_euler_to_vector.inputs[2])
    #group_input.Color Input -> set_material.Material
    pixel_nodes.links.new(group_input.outputs[1], set_material.inputs[2])
    #join_geometry.Geometry -> group_output.Geometry
    pixel_nodes.links.new(join_geometry.outputs[0], group_output.inputs[0])
    #group_input.Geometry -> set_material_001.Geometry
    pixel_nodes.links.new(group_input.outputs[0], set_material_001.inputs[0])
    #normal.Normal -> vector_math.Vector
    pixel_nodes.links.new(normal.outputs[0], vector_math.inputs[0])
    #instance_on_points.Instances -> realize_instances.Geometry
    pixel_nodes.links.new(instance_on_points.outputs[0], realize_instances.inputs[0])
    #join_geometry.Geometry -> viewer.Geometry
    pixel_nodes.links.new(join_geometry.outputs[0], viewer.inputs[0])
    #vector_math.Vector -> viewer.Value
    pixel_nodes.links.new(vector_math.outputs[0], viewer.inputs[1])
    #group_input.Geometry -> instance_on_points.Points
    pixel_nodes.links.new(group_input.outputs[0], instance_on_points.inputs[0])
    #vector_math.Vector -> set_position.Offset
    pixel_nodes.links.new(vector_math.outputs[0], set_position.inputs[3])
    #set_position.Geometry -> join_geometry.Geometry
    pixel_nodes.links.new(set_position.outputs[0], join_geometry.inputs[0])
    #realize_instances.Geometry -> set_position.Geometry
    pixel_nodes.links.new(realize_instances.outputs[0], set_position.inputs[0])
    #set_material_001.Geometry -> join_geometry.Geometry
    pixel_nodes.links.new(set_material_001.outputs[0], join_geometry.inputs[0])
    return pixel_nodes




image_path = "C:/Users/maxd5/Documents/GitHub/deformable-pov-display/scripts (python)/troposkein modeling/images/low_morphskein.jpg"

#image_path = "C:/Users/maxd5/Documents/GitHub/deformable-pov-display/scripts (python)/troposkein modeling/images/test.png"

scale_factor = 0.1
render_persistence = False
render_projection = False
render_hardware = True
isosphere_radius = 0.3
bloom_enabled = True


# LOOP OVER CONFIGURATIONS
camera_x_step_per_config = 85
camera_y_step_per_config = 115


config0 = {
    "position": [0, 0, 0],
    "rendering_type": constants.PERSISTENCE_TYPE,
    'angular_speed':constants.DEFAULT_ANGULAR_SPEED,
    'rope_length':203,
    'pole_height':200,
    'hole_radius':constants.DEFAULT_HOLE_RADIUS,
    'strip_mass':constants.DEFAULT_STRIP_MASS/1000.0,
    'vertical_pixel_pitch':constants.DEFAULT_PIXEL_PITCH,
    'horizontal_pixel_pitch':constants.DEFAULT_PIXEL_PITCH,
    'top_rope_length_offset':constants.DEFAULT_TOP_ROPE_LENGTH_OFFSET,
}

config1 = {
    "position": [35, 0, 0],
    "rendering_type": constants.PERSISTENCE_TYPE,
    'angular_speed':constants.DEFAULT_ANGULAR_SPEED,
    'rope_length':497,
    'pole_height':494,
    'hole_radius':constants.DEFAULT_HOLE_RADIUS,
    'strip_mass':constants.DEFAULT_STRIP_MASS/1000.0,
    'vertical_pixel_pitch':constants.DEFAULT_PIXEL_PITCH,
    'horizontal_pixel_pitch':constants.DEFAULT_PIXEL_PITCH,
    'top_rope_length_offset':constants.DEFAULT_TOP_ROPE_LENGTH_OFFSET
}

config2 = {
    "position": [105, 0, 0],
    "rendering_type": constants.PERSISTENCE_TYPE,
    'angular_speed':constants.DEFAULT_ANGULAR_SPEED,
    'rope_length':742,
    'pole_height':200,
    'hole_radius':constants.DEFAULT_HOLE_RADIUS,
    'strip_mass':constants.DEFAULT_STRIP_MASS/1000.0,
    'vertical_pixel_pitch':constants.DEFAULT_PIXEL_PITCH,
    'horizontal_pixel_pitch':constants.DEFAULT_PIXEL_PITCH,
    'top_rope_length_offset':constants.DEFAULT_TOP_ROPE_LENGTH_OFFSET
}

config3 = {
    "position": [195, 0, 0],
    "rendering_type": constants.PERSISTENCE_TYPE,
    'angular_speed':constants.DEFAULT_ANGULAR_SPEED,
    'rope_length':742,
    'pole_height':494,
    'hole_radius':constants.DEFAULT_HOLE_RADIUS,
    'strip_mass':constants.DEFAULT_STRIP_MASS/1000.0,
    'vertical_pixel_pitch':constants.DEFAULT_PIXEL_PITCH,
    'horizontal_pixel_pitch':constants.DEFAULT_PIXEL_PITCH,
    'top_rope_length_offset':constants.DEFAULT_TOP_ROPE_LENGTH_OFFSET
}
configs = [config0, config1, config2, config3]

# x_axis_types = [constants.PERSISTENCE_TYPE, constants.ARRAY_TYPE, constants.PROJECTION_TYPE]
# y_axis_configs = 
nb_initial_configs = len(configs)
initial_vertical_pixel_resolution = None
initial_horizontal_pixel_resolution = None


# S=constants.MAX_ROPE_LENGTH
# L=constants.MAX_POLE_HEIGHT
# O=constants.DEFAULT_HOLE_RADIUS
# D=constants.DEFAULT_STRIP_MASS/1000.0
# Pv=constants.DEFAULT_PIXEL_PITCH
# Ph=constants.DEFAULT_PIXEL_PITCH
# T=constants.DEFAULT_TOP_ROPE_LENGTH_OFFSET

# worst case
# W=constants.DEFAULT_ANGULAR_SPEED
# S=constants.MAX_ROPE_LENGTH
# L=constants.MAX_POLE_HEIGHT
# O=constants.DEFAULT_HOLE_RADIUS
# D=constants.DEFAULT_STRIP_MASS/1000.0
# Pv=constants.DEFAULT_PIXEL_PITCH
# Ph=constants.DEFAULT_PIXEL_PITCH
# T=constants.DEFAULT_TOP_ROPE_LENGTH_OFFSET


# Clear existing mesh data>
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type='MESH')
bpy.ops.object.delete()


def look_at(obj_camera, point):
    loc_camera = obj_camera.matrix_world.to_translation()
    direction = point - loc_camera
    rot_quat = direction.to_track_quat('-Z', 'Y')
    obj_camera.rotation_euler = rot_quat.to_euler()


def render_background(root_obj, points):
    origins =  np.zeros(points.shape)
    origins[:, 2]  = points[:, 2] 
    directions  = points - origins
    magnitudes = np.linalg.norm(directions, axis=1)
    scale_factors = 1 - 0.5/magnitudes
    new_points = origins + directions * scale_factors[:, np.newaxis]

    nb_rows = max_vertical_resolution
    nb_cols = int(points.shape[0]//max_vertical_resolution)
    triangles = []
    for i in range(new_points.shape[0]):
        col = i // max_vertical_resolution
        row = i % max_vertical_resolution
        if row < nb_rows - 1 and col < nb_cols :
            top_left = i % new_points.shape[0]
            top_right = (i + nb_rows) % new_points.shape[0]
            bottom_left = (i + 1) % new_points.shape[0]
            bottom_right = (i + nb_rows + 1) % new_points.shape[0]
            triangles.append([top_left, bottom_left, top_right])
            triangles.append([bottom_left, bottom_right, top_right])
    
    # Define the geometry
    verts = new_points
    edges = [] 
    faces = triangles
    # Create a new mesh and object
    ob_name = f"background_{config_id}"
    mesh = bpy.data.meshes.new(ob_name + "_mesh")
    obj  = bpy.data.objects.new(ob_name, mesh)

    # Add a Solidify modifier to the object
    solidify_modifier  = obj.modifiers.new("Solidify", 'SOLIDIFY')

    # # Set the parameters for the Solidify modifier
    solidify_modifier.thickness = isosphere_radius * 2       # Adjust the thickness value
    solidify_modifier.offset = -isosphere_radius * 2           # Adjust the offset value

    # Link object to scene
    scene = bpy.context.collection
    scene.objects.link(obj)

    # Create mesh from given vertices, edges, and faces
    mesh.from_pydata(verts, edges, faces)
    mesh.update()

    # Switch to object mode
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    #bpy.ops.object.parent_set( type = 'VERTEX', keep_transform = True )

    # Create a new material
    material = bpy.data.materials.new(name="RandomColorMaterial")
    material.use_nodes = True
    material.blend_method = "OPAQUE"
    nodes = material.node_tree.nodes
    links = material.node_tree.links

    # Clear all nodes to start fresh
    for node in nodes:
        nodes.remove(node)

    # Add new nodes
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    diffuse_node = nodes.new(type='ShaderNodeEmission')

    # Link nodes
    links.new(diffuse_node.outputs['Emission'], output_node.inputs['Surface'])

    # Assign material to the object
    if obj.data.materials:
        obj.data.materials[0] = material
    else:
        obj.data.materials.append(material)


    # Create vertex color layer
    color_layer = mesh.vertex_colors.new(name="Col")

    # Set random colors for each face
    mesh.polygons.foreach_set("use_smooth", [False] * len(mesh.polygons))  # Disable smoothing for all faces

    # if col % 2 == 0:
    #     random_color[3] = 1.0
    # else:
    #     random_color[3] = 0.0

    for i, poly in enumerate(mesh.polygons):
        #>random_color = image_pixels[i]
        color_layer = mesh.vertex_colors.get("Col") or mesh.vertex_colors.new(name="Col")
        for loop_index in poly.loop_indices:
            color_layer.data[loop_index].color = [0, 0, 0, 1.0] #random_color

    # Update the mesh with new colors
    mesh.update()

    # Update material to use vertex colors
    diffuse_node.inputs['Color'].default_value = (1, 1, 1, 1)
    attribute_node = nodes.new(type='ShaderNodeAttribute')
    attribute_node.attribute_name = "Col"
    links.new(attribute_node.outputs['Color'], diffuse_node.inputs['Color'])
    obj.parent = root_obj

def geometry_node_strip(scale_factor, nb_duplicates):
    geometry_nodes = bpy.data.node_groups.new(type = 'GeometryNodeTree', name = "Geometry Nodes")

    #initialize geometry_nodes nodes
    #node Math
    math = geometry_nodes.nodes.new("ShaderNodeMath")
    math.name = "Math"
    math.operation = 'DIVIDE'
    math.use_clamp = False
    #Value
    math.inputs[0].default_value = 6.28000020980835
    #Value_002
    math.inputs[2].default_value = 0.5
    
    #node Position
    position = geometry_nodes.nodes.new("GeometryNodeInputPosition")
    position.name = "Position"
    
    #node Math.001
    math_001 = geometry_nodes.nodes.new("ShaderNodeMath")
    math_001.name = "Math.001"
    math_001.operation = 'MULTIPLY'
    math_001.use_clamp = False
    #Value_002
    math_001.inputs[2].default_value = 0.5
    
    #node Vector Rotate
    vector_rotate = geometry_nodes.nodes.new("ShaderNodeVectorRotate")
    vector_rotate.name = "Vector Rotate"
    vector_rotate.invert = False
    vector_rotate.rotation_type = 'AXIS_ANGLE'
    #Center
    vector_rotate.inputs[1].default_value = (0.0, 0.0, 0.0)
    #Axis
    vector_rotate.inputs[2].default_value = (0.0, 0.0, 1.0)
    #Rotation
    vector_rotate.inputs[4].default_value = (0.0, 0.0, 0.0)
    
    #node Group Input
    group_input = geometry_nodes.nodes.new("NodeGroupInput")
    group_input.name = "Group Input"
    #geometry_nodes inputs
    #input Geometry
    geometry_nodes.inputs.new('NodeSocketGeometry', "Geometry")
    geometry_nodes.inputs[0].attribute_domain = 'POINT'
    
    
    
    #node Value
    value = geometry_nodes.nodes.new("ShaderNodeValue")
    value.name = "Value"
    
    value.outputs[0].default_value = nb_duplicates
    #node Duplicate Elements
    duplicate_elements = geometry_nodes.nodes.new("GeometryNodeDuplicateElements")
    duplicate_elements.name = "Duplicate Elements"
    duplicate_elements.domain = 'FACE'
    #Selection
    duplicate_elements.inputs[1].default_value = True
    
    #node Set Position
    set_position = geometry_nodes.nodes.new("GeometryNodeSetPosition")
    set_position.name = "Set Position"
    #Selection
    set_position.inputs[1].default_value = True
    #Offset
    set_position.inputs[3].default_value = (0.0, 0.0, 0.0)
    
    #node Group Output
    group_output = geometry_nodes.nodes.new("NodeGroupOutput")
    group_output.name = "Group Output"
    group_output.is_active_output = True
    #geometry_nodes outputs
    #output Geometry
    geometry_nodes.outputs.new('NodeSocketGeometry', "Geometry")
    geometry_nodes.outputs[0].attribute_domain = 'POINT'
    
    
    
    #node Scale Elements
    scale_elements = geometry_nodes.nodes.new("GeometryNodeScaleElements")
    scale_elements.name = "Scale Elements"
    scale_elements.domain = 'FACE'
    scale_elements.scale_mode = 'SINGLE_AXIS'
    #Selection
    scale_elements.inputs[1].default_value = True
    #Scale
    scale_elements.inputs[2].default_value = scale_factor
    #Center
    scale_elements.inputs[3].default_value = (0.0, 0.0, 0.0)
    #Axis
    scale_elements.inputs[4].default_value = (1.0, 0.0, 0.0)
    
    
    
    #Set locations
    math.location = (-183.3560333251953, 47.284393310546875)
    position.location = (230.31813049316406, 109.1307373046875)
    math_001.location = (67.2501449584961, 48.29158401489258)
    vector_rotate.location = (432.0482177734375, 71.58592224121094)
    group_input.location = (-803.176025390625, 312.673095703125)
    value.location = (-493.3218688964844, 220.4699249267578)
    duplicate_elements.location = (-146.82913208007812, 460.0428466796875)
    set_position.location = (749.9645385742188, 242.97557067871094)
    group_output.location = (1159.5869140625, 190.27102661132812)
    scale_elements.location = (-434.920654296875, 559.2175903320312)
    
    #Set dimensions
    math.width, math.height = 140.0, 100.0
    position.width, position.height = 140.0, 100.0
    math_001.width, math_001.height = 140.0, 100.0
    vector_rotate.width, vector_rotate.height = 140.0, 100.0
    group_input.width, group_input.height = 140.0, 100.0
    value.width, value.height = 140.0, 100.0
    duplicate_elements.width, duplicate_elements.height = 140.0, 100.0
    set_position.width, set_position.height = 140.0, 100.0
    group_output.width, group_output.height = 140.0, 100.0
    scale_elements.width, scale_elements.height = 140.0, 100.0
    
    #initialize geometry_nodes links
    #duplicate_elements.Geometry -> set_position.Geometry
    geometry_nodes.links.new(duplicate_elements.outputs[0], set_position.inputs[0])
    #value.Value -> duplicate_elements.Amount
    geometry_nodes.links.new(value.outputs[0], duplicate_elements.inputs[2])
    #value.Value -> math.Value
    geometry_nodes.links.new(value.outputs[0], math.inputs[1])
    #math.Value -> math_001.Value
    geometry_nodes.links.new(math.outputs[0], math_001.inputs[1])
    #duplicate_elements.Duplicate Index -> math_001.Value
    geometry_nodes.links.new(duplicate_elements.outputs[1], math_001.inputs[0])
    #position.Position -> vector_rotate.Vector
    geometry_nodes.links.new(position.outputs[0], vector_rotate.inputs[0])
    #math_001.Value -> vector_rotate.Angle
    geometry_nodes.links.new(math_001.outputs[0], vector_rotate.inputs[3])
    #vector_rotate.Vector -> set_position.Position
    geometry_nodes.links.new(vector_rotate.outputs[0], set_position.inputs[2])
    #group_input.Geometry -> scale_elements.Geometry
    geometry_nodes.links.new(group_input.outputs[0], scale_elements.inputs[0])
    #scale_elements.Geometry -> duplicate_elements.Geometry
    geometry_nodes.links.new(scale_elements.outputs[0], duplicate_elements.inputs[0])
    #set_position.Geometry -> group_output.Geometry
    geometry_nodes.links.new(set_position.outputs[0], group_output.inputs[0])
    return geometry_nodes


#initialize PersistenceMaterial node group
def persistencematerial_node_group(persistence_material):

	persistencematerial = persistence_material.node_tree
	#start with a clean node tree
	for node in persistencematerial.nodes:
		persistencematerial.nodes.remove(node)
	#initialize persistencematerial nodes
	#node Material Output
	material_output = persistencematerial.nodes.new("ShaderNodeOutputMaterial")
	material_output.name = "Material Output"
	material_output.is_active_output = True
	material_output.target = 'ALL'
	#Displacement
	material_output.inputs[2].default_value = (0.0, 0.0, 0.0)
	#Thickness
	material_output.inputs[3].default_value = 0.0
	
	#node Attribute
	attribute = persistencematerial.nodes.new("ShaderNodeAttribute")
	attribute.name = "Attribute"
	attribute.attribute_name = "Col"
	attribute.attribute_type = 'GEOMETRY'
	
	#node Transparent BSDF
	transparent_bsdf = persistencematerial.nodes.new("ShaderNodeBsdfTransparent")
	transparent_bsdf.name = "Transparent BSDF"
	#Color
	transparent_bsdf.inputs[0].default_value = (1.0, 1.0, 1.0, 1.0)
	#Weight
	transparent_bsdf.inputs[1].default_value = 0.0
	
	#node Emission
	emission = persistencematerial.nodes.new("ShaderNodeEmission")
	emission.name = "Emission"
	#Strength
	emission.inputs[1].default_value = 3.0
	#Weight
	emission.inputs[2].default_value = 0.0
	
	#node Mix Shader
	mix_shader = persistencematerial.nodes.new("ShaderNodeMixShader")
	mix_shader.name = "Mix Shader"
	
	
	#Set locations
	material_output.location = (483.7955322265625, -63.492210388183594)
	attribute.location = (-299.18896484375, -54.20933532714844)
	transparent_bsdf.location = (-30.146957397460938, -268.1473388671875)
	emission.location = (47.3663330078125, -19.8626708984375)
	mix_shader.location = (274.7105712890625, -152.33888244628906)
	
	#Set dimensions
	material_output.width, material_output.height = 140.0, 100.0
	attribute.width, attribute.height = 140.0, 100.0
	transparent_bsdf.width, transparent_bsdf.height = 140.0, 100.0
	emission.width, emission.height = 140.0, 100.0
	mix_shader.width, mix_shader.height = 140.0, 100.0
	
	#initialize persistencematerial links
	#attribute.Color -> emission.Color
	persistencematerial.links.new(attribute.outputs[0], emission.inputs[0])
	#mix_shader.Shader -> material_output.Surface
	persistencematerial.links.new(mix_shader.outputs[0], material_output.inputs[0])
	#attribute.Alpha -> mix_shader.Fac
	persistencematerial.links.new(attribute.outputs[3], mix_shader.inputs[0])
	#transparent_bsdf.BSDF -> mix_shader.Shader
	persistencematerial.links.new(transparent_bsdf.outputs[0], mix_shader.inputs[1])
	#emission.Emission -> mix_shader.Shader
	persistencematerial.links.new(emission.outputs[0], mix_shader.inputs[2])
	return persistencematerial




def projectionmaterial_node_group(projection_material, emission_strength=1):
    nodes = projection_material.node_tree.nodes
    links = projection_material.node_tree.links

    # Clear all nodes to start fresh
    for node in nodes:
        nodes.remove(node)

    # Add new nodes
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    diffuse_node = nodes.new(type='ShaderNodeEmission')
    diffuse_node.inputs[1].default_value = emission_strength

    # Link nodes
    links.new(diffuse_node.outputs['Emission'], output_node.inputs['Surface'])
    # Update material to use vertex colors
    diffuse_node.inputs['Color'].default_value = (1, 1, 1, 1)
    attribute_node = nodes.new(type='ShaderNodeAttribute')
    attribute_node.attribute_name = "Col"
    links.new(attribute_node.outputs['Color'], diffuse_node.inputs['Color'])

    return projection_material



def render_strips(root_obj, points):
    origins =  np.zeros(points.shape)
    origins[:, 2]  = points[:, 2] 
    directions  = points - origins
    magnitudes = np.linalg.norm(directions, axis=1)
    scale_factors = 1 - (2 * isosphere_radius)/magnitudes
    new_points = origins + directions * scale_factors[:, np.newaxis]

    nb_rows = max_vertical_resolution
    nb_cols = int(points.shape[0]//max_vertical_resolution)
    triangles = []
    for i in range(new_points.shape[0]):
        col = i // max_vertical_resolution
        row = i % max_vertical_resolution
        if col == 1 and row == 0:
            break
        if row < nb_rows - 1 and col < nb_cols :
            top_left = i % new_points.shape[0]
            top_right = (i + nb_rows) % new_points.shape[0]
            bottom_left = (i + 1) % new_points.shape[0]
            bottom_right = (i + nb_rows + 1) % new_points.shape[0]
            triangles.append([top_left, bottom_left, top_right])
            triangles.append([bottom_left, bottom_right, top_right])
    # Define the geometry
    verts = new_points
    edges = [] 
    faces = triangles
    # Create a new mesh and object
    ob_name = f"background_{config_id}"
    mesh = bpy.data.meshes.new(ob_name + "_mesh")
    obj  = bpy.data.objects.new(ob_name, mesh)

    # Add a Solidify modifier to the object
    solidify_modifier  = obj.modifiers.new("Solidify", 'SOLIDIFY')

    # # Set the parameters for the Solidify modifier
    solidify_modifier.thickness = isosphere_radius * 2       # Adjust the thickness value
    solidify_modifier.offset = -isosphere_radius * 2           # Adjust the offset value

    # Link object to scene
    scene = bpy.context.collection
    scene.objects.link(obj)

    # Create mesh from given vertices, edges, and faces
    mesh.from_pydata(verts, edges, faces)
    mesh.update()

    # Switch to object mode
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    #bpy.ops.object.parent_set( type = 'VERTEX', keep_transform = True )

    # Create a new material
    material = bpy.data.materials.new(name="RandomColorMaterial")
    material.use_nodes = True
    material.blend_method = "OPAQUE"
    nodes = material.node_tree.nodes
    links = material.node_tree.links

    # Clear all nodes to start fresh
    for node in nodes:
        nodes.remove(node)

    # Add new nodes
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    diffuse_node = nodes.new(type='ShaderNodeEmission')

    # Link nodes
    links.new(diffuse_node.outputs['Emission'], output_node.inputs['Surface'])

    # Assign material to the object
    if obj.data.materials:
        obj.data.materials[0] = material
    else:
        obj.data.materials.append(material)


    # Create vertex color layer
    color_layer = mesh.vertex_colors.new(name="Col")

    # Set random colors for each face
    mesh.polygons.foreach_set("use_smooth", [False] * len(mesh.polygons))  # Disable smoothing for all faces

    # if col % 2 == 0:
    #     random_color[3] = 1.0
    # else:
    #     random_color[3] = 0.0

    for i, poly in enumerate(mesh.polygons):
        #>random_color = image_pixels[i]
        color_layer = mesh.vertex_colors.get("Col") or mesh.vertex_colors.new(name="Col")
        for loop_index in poly.loop_indices:
            color_layer.data[loop_index].color = [0, 0, 0, 0.5] #random_color

    # Update the mesh with new colors
    mesh.update()

    # Update material to use vertex colors
    diffuse_node.inputs['Color'].default_value = (1, 1, 1, 1)
    attribute_node = nodes.new(type='ShaderNodeAttribute')
    attribute_node.attribute_name = "Col"
    links.new(attribute_node.outputs['Color'], diffuse_node.inputs['Color'])

    mod = obj.modifiers.new("Geo Node Modifier", type='NODES')
    mod.node_group = geometry_node_strip(0.5, max_horizontal_resolution)
    obj.parent = root_obj


def render_faces(root_obj, points, use_backface_culling=False, emission_strength=1.0):
    # open image
    image_bgr = cv2.imread(image_path)
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    image_pixels = []
    image_start_v = image_rgb.shape[0]/2.0 - max_vertical_resolution/2.0
    image_start_h = image_rgb.shape[1]/2.0 - max_horizontal_resolution/2.0
    nb_rows = max_vertical_resolution
    nb_cols = int(points.shape[0]//max_vertical_resolution)
    triangles = []
    image_pixels = []
    for i in range(points.shape[0]):
        col = i // max_vertical_resolution
        row = i % max_vertical_resolution
        if row < nb_rows - 1 and col < nb_cols :
            top_left = i % points.shape[0]
            top_right = (i + nb_rows) % points.shape[0]
            bottom_left = (i + 1) % points.shape[0]
            bottom_right = (i + nb_rows + 1) % points.shape[0]
            triangles.append([top_left, bottom_left, top_right])
            triangles.append([bottom_left, bottom_right, top_right])
            if row >= 0 and row < image_rgb.shape[0] and col >= 0 and col < image_rgb.shape[1]:
                r, g, b = image_rgb[int(image_start_v + row), int(image_start_h + col)]
                image_pixels.append([r/255.0, g/255.0, b/255.0, 1.0])
                image_pixels.append([r/255.0, g/255.0, b/255.0, 1.0])
                # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
                # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
            else:
                image_pixels.append([0.0, 0.0, 0.0, 0.0])
                image_pixels.append([0.0, 0.0, 0.0, 0.0])
    
    print("first pixel:", image_pixels[0])
    # Define the geometry
    verts = points
    edges = [] 
    faces = triangles
    # Create a new mesh and object
    ob_name = f"troposkein_{config_id}"
    mesh = bpy.data.meshes.new(ob_name + "_mesh")
    obj  = bpy.data.objects.new(ob_name, mesh)

    # Link object to scene
    scene = bpy.context.collection
    scene.objects.link(obj)

    # Create mesh from given vertices, edges, and faces
    mesh.from_pydata(verts, edges, faces)
    mesh.update()

    # Switch to object mode
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    #bpy.ops.object.parent_set( type = 'VERTEX', keep_transform = True )

    # Create vertex color layer
    color_layer = mesh.vertex_colors.new(name="Col")

    # Set random colors for each face
    mesh.polygons.foreach_set("use_smooth", [False] * len(mesh.polygons))  # Disable smoothing for all faces
    for i, poly in enumerate(mesh.polygons):
        #random_color = [random.random() for _ in range(3)] + [1]
        random_color = image_pixels[i]
        color_layer = mesh.vertex_colors.get("Col") or mesh.vertex_colors.new(name="Col")
        for loop_index in poly.loop_indices:
            color_layer.data[loop_index].color = random_color

    # Update the mesh with new colors
    mesh.update()

    # Create a new material
    material = bpy.data.materials.new(name="ProjectionMaterial")
    material.use_nodes = True
    material.use_backface_culling = use_backface_culling 
    material = projectionmaterial_node_group(material, emission_strength)
    # Assign material to the object
    if obj.data.materials:
        obj.data.materials[0] = material
    else:
        obj.data.materials.append(material)
    obj.parent = root_obj

def render_points(root_obj, points, use_backface_culling=False, background_alpha=0.5, emission_strength=5.0):
    # open image
    image_bgr = cv2.imread(image_path)
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    image_pixels = []
    image_start_v = image_rgb.shape[0]/2.0 - max_vertical_resolution/2.0
    image_start_h = image_rgb.shape[1]/2.0 - max_horizontal_resolution/2.0
    nb_rows = max_vertical_resolution
    nb_cols = int(points.shape[0]//max_vertical_resolution)
    triangles = []
    image_pixels = []
    for i in range(points.shape[0]):
        col = i // max_vertical_resolution
        row = i % max_vertical_resolution
        if row < nb_rows - 1 and col < nb_cols :
            top_left = i % points.shape[0]
            top_right = (i + nb_rows) % points.shape[0]
            bottom_left = (i + 1) % points.shape[0]
            bottom_right = (i + nb_rows + 1) % points.shape[0]
            triangles.append([top_left, bottom_left, top_right])
            triangles.append([bottom_left, bottom_right, top_right])
            if row >= 0 and row < image_rgb.shape[0] and col >= 0 and col < image_rgb.shape[1]:
                r, g, b = image_rgb[int(image_start_v + row), int(image_start_h + col)]
                image_pixels.append([r/255.0, g/255.0, b/255.0, 1.0])
                image_pixels.append([r/255.0, g/255.0, b/255.0, 1.0])
                # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
                # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
            else:
                image_pixels.append([0.0, 0.0, 0.0, 0.0])
                image_pixels.append([0.0, 0.0, 0.0, 0.0])
    
    print("first pixel:", image_pixels[0])
    # Define the geometry
    verts = points
    edges = [] 
    faces = triangles
    # Create a new mesh and object
    ob_name = f"troposkein_{config_id}"
    mesh = bpy.data.meshes.new(ob_name + "_mesh")
    obj  = bpy.data.objects.new(ob_name, mesh)

    # Link object to scene
    scene = bpy.context.collection
    scene.objects.link(obj)

    # Create mesh from given vertices, edges, and faces
    mesh.from_pydata(verts, edges, faces)
    mesh.update()

    # Switch to object mode
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    #bpy.ops.object.parent_set( type = 'VERTEX', keep_transform = True )

    # Create a new material
    pixel_mat = bpy.data.materials.new(name = "PixelMaterial")
    pixel_mat.use_nodes = True
    pixel_mat.use_backface_culling = use_backface_culling 

    # Assign material to the object
    if obj.data.materials:
        obj.data.materials[0] = pixel_mat
    else:
        obj.data.materials.append(pixel_mat)

    background_mat = bpy.data.materials.new(name = "BackgroundMaterial")
    background_mat.use_nodes = True
    background_mat.blend_method = "BLEND"
    obj.data.materials.append(background_mat)


    # Create vertex color layer
    color_layer = mesh.vertex_colors.new(name="Col")

    # Set random colors for each face
    mesh.polygons.foreach_set("use_smooth", [False] * len(mesh.polygons))  # Disable smoothing for all faces
    for i, poly in enumerate(mesh.polygons):
        #random_color = [random.random() for _ in range(3)] + [1]
        random_color = image_pixels[i]
        color_layer = mesh.vertex_colors.get("Col") or mesh.vertex_colors.new(name="Col")
        for loop_index in poly.loop_indices:
            color_layer.data[loop_index].color = random_color

    #      if len(mesh.vertices) != len(color):
    #     raise ValueError(
    #         f"Got {len(mesh.vertices)} vertices and {len(color)} color values"
    #     )

    # if Constants.MARKER_COLOR not in mesh.attributes:
    #     mesh.attributes.new(
    #         name=Constants.MARKER_COLOR, type="FLOAT_COLOR", domain="POINT"
    #     )
    # mesh.attributes[Constants.MARKER_COLOR].data.foreach_set("color", color.reshape(-1))

    # Update the mesh with new colors
    mesh.update()
    # pixel nodes
    pixel_mat_node = pixelmaterial_node_group(pixel_mat, emission_strength)
    background_mat_node = backgroundmaterial_node_group(background_mat, background_alpha)

    mod = obj.modifiers.new("Pixel Node Modifier", type='NODES')
    mod.node_group = pixel_nodes_node_group(pixel_mat, background_mat)
    obj.parent = root_obj
    


def render_persistence(root_obj, points, use_backface_culling=False, background_alpha=0.5, emission_strength=5.0):
    # open image
    image_bgr = cv2.imread(image_path)
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    image_pixels = []
    image_start_v = image_rgb.shape[0]/2.0 - (max_vertical_resolution//2)/2.0
    image_start_h = image_rgb.shape[1]/2.0 - max_horizontal_resolution/2.0
    nb_rows = max_vertical_resolution
    nb_cols = int(points.shape[0]//max_vertical_resolution)
    triangles = []
    image_pixels = []
    for i in range(points.shape[0]):
        col = i // max_vertical_resolution
        row = i % max_vertical_resolution
        if row%2 and row < nb_rows - 1 and col < nb_cols :
            top_left = i % points.shape[0]
            top_right = (i + nb_rows) % points.shape[0]
            bottom_left = (i + 1) % points.shape[0]
            bottom_right = (i + nb_rows + 1) % points.shape[0]
            triangles.append([top_left, bottom_left, top_right])
            triangles.append([bottom_left, bottom_right, top_right])
            if row >= 0 and row < image_rgb.shape[0] and col >= 0 and col < image_rgb.shape[1]:
                r, g, b = image_rgb[int(image_start_v + row//2), int(image_start_h + col)]
                led_rgba = compute_led_rgba(r,g,b)
                image_pixels.append(led_rgba)
                image_pixels.append(led_rgba)
                # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
                # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
            else:
                image_pixels.append([0.0, 0.0, 0.0, 0.0])
                image_pixels.append([0.0, 0.0, 0.0, 0.0])
    
    print("first pixel:", image_pixels[0])
    # Define the geometry
    verts = points
    edges = [] 
    faces = triangles
    # Create a new mesh and object
    ob_name = f"troposkein_{config_id}"
    mesh = bpy.data.meshes.new(ob_name + "_mesh")
    obj  = bpy.data.objects.new(ob_name, mesh)

    # Link object to scene
    scene = bpy.context.collection
    scene.objects.link(obj)

    # Create mesh from given vertices, edges, and faces
    mesh.from_pydata(verts, edges, faces)
    mesh.update()

    # Switch to object mode
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    #bpy.ops.object.parent_set( type = 'VERTEX', keep_transform = True )

    # Create vertex color layer
    color_layer = mesh.vertex_colors.new(name="Col")

    # Set random colors for each face
    mesh.polygons.foreach_set("use_smooth", [False] * len(mesh.polygons))  # Disable smoothing for all faces
    for i, poly in enumerate(mesh.polygons):
        #random_color = [random.random() for _ in range(3)] + [1]
        random_color = image_pixels[i]
        color_layer = mesh.vertex_colors.get("Col") or mesh.vertex_colors.new(name="Col")
        for loop_index in poly.loop_indices:
            color_layer.data[loop_index].color = random_color

    # Update the mesh with new colors
    mesh.update()

    # Create a new material
    material = bpy.data.materials.new(name="PersistenceMaterial")
    material.use_nodes = True
    material.use_backface_culling = use_backface_culling 
    material.blend_method = "BLEND"
    persistencematerial_node_group(material)
    # Assign material to the object
    if obj.data.materials:
        obj.data.materials[0] = material
    else:
        obj.data.materials.append(material)
    obj.parent = root_obj
    


for config_id, config in enumerate(configs):
    config_type = config['rendering_type']
    config_position = config['position']
 
    pixels_x, pixels_y, pixels_z,max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein3D(
        config['angular_speed'], 
        config['rope_length'], 
        config['pole_height'], 
        config['hole_radius'], 
        config['strip_mass'], 
        config['vertical_pixel_pitch'], 
        config['horizontal_pixel_pitch'], 
        config['top_rope_length_offset'], 
        verbose=False   
    )

    if config_id < nb_initial_configs:
        if config_id == 0:
            array_config = config.copy()
            array_config["rendering_type"] = constants.ARRAY_TYPE
            array_config["position"] = [config_position[0], constants.ARRAY_TYPE * camera_y_step_per_config, config_position[2]]
            configs.append(array_config)
            projection_config = config.copy()
            projection_config["rendering_type"] = constants.PROJECTION_TYPE
            projection_config["position"] = [config_position[0], constants.PROJECTION_TYPE * camera_y_step_per_config, config_position[2]]
            configs.append(projection_config)
            initial_vertical_pixel_resolution = max_vertical_resolution
            initial_horizontal_pixel_resolution = max_horizontal_resolution

        else:
            array_config = config.copy()
            array_config["rendering_type"] = constants.ARRAY_TYPE
            array_config["position"] = [config_position[0], constants.ARRAY_TYPE * camera_y_step_per_config, config_position[2]]
            array_config['horizontal_pixel_pitch'] =  (max_diameter * math.pi) / initial_horizontal_pixel_resolution # pitch
            array_config['vertical_pixel_pitch'] =  array_config["rope_length"] / initial_vertical_pixel_resolution
            configs.append(array_config)
            projection_config = config.copy()
            projection_config["rendering_type"] = constants.PROJECTION_TYPE
            projection_config["position"] = [config_position[0], constants.PROJECTION_TYPE * camera_y_step_per_config, config_position[2]]
            projection_config['horizontal_pixel_pitch'] =  (max_diameter * math.pi) / initial_horizontal_pixel_resolution # pitch
            projection_config['vertical_pixel_pitch'] =  array_config["rope_length"] / initial_vertical_pixel_resolution
            configs.append(projection_config)

    if config_type == constants.PERSISTENCE_TYPE:
        vertical_pixel_resolution = config['pole_height']/config['vertical_pixel_pitch']
        new_vertical_pixel_pitch = config['pole_height']/(vertical_pixel_resolution * 2  - 1)
        pixels_x, pixels_y, pixels_z,max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein3D(
            config['angular_speed'], 
            config['rope_length'], 
            config['pole_height'], 
            config['hole_radius'], 
            config['strip_mass'], 
            new_vertical_pixel_pitch, 
            config['horizontal_pixel_pitch'], 
            config['top_rope_length_offset'], 
            verbose=False   
        )


    points = np.column_stack((pixels_x, pixels_y, pixels_z)) * scale_factor



    t = time.time()

    
    print("CONFIGURATION TYPE:", config_type)
    print(tuple(config_position))
    bpy.ops.object.empty_add(location=tuple(config_position)) 
    parent_obj = bpy.context.active_object
    parent_obj.name = f"{constants.renderingTypeToString(config_type)}_{config_id}" #bpy.data.objects.new(f"{config['rendering_type']}_{config_id}", None)

    match config_type:
        case constants.ARRAY_TYPE:
            render_points(parent_obj, points, use_backface_culling=True, background_alpha=0.0, emission_strength=5.0)
        case constants.PROJECTION_TYPE:
            render_faces(parent_obj, points, use_backface_culling=True, emission_strength=0.6)
        case constants.PERSISTENCE_TYPE:    
            #render_points(parent_obj, points, use_backface_culling=True, background_alpha=0.0, emission_strength=5.0)
            render_persistence(parent_obj, points, use_backface_culling=True, background_alpha=0.0, emission_strength=3.0)
        case _:
            break 



# Enable use of nodes for the world
bpy.context.scene.world.use_nodes = True
world_nodes = bpy.context.scene.world.node_tree.nodes

# Clear default nodes
for node in world_nodes:
    world_nodes.remove(node)

# Add background node and set the color to white
background_node = world_nodes.new(type='ShaderNodeBackground')
background_node.inputs['Color'].default_value = (0.02, 0.02, 0.02, 1)  # White color
#background_node.inputs['Color'].default_value = (0.0, 0.0, 0.0, 0.0)  # White color

# Add output node and link it to the background nodeF
new_output_node = world_nodes.new(type='ShaderNodeOutputWorld')
bpy.context.scene.world.node_tree.links.new(background_node.outputs['Background'], new_output_node.inputs['Surface'])


obj_camera = bpy.data.objects["Camera"]
#obj_other = bpy.data.objects["Cube"]

# set the camera's location and rotation
obj_camera.location = (110, -11, 150)
obj_camera.rotation_euler = (50.0/180.0 * math.pi, 0, 0)

# Set the camera to orthographic mode
obj_camera.data.type = 'ORTHO'
obj_camera.data.ortho_scale = 400

# Set the clipping distances
obj_camera.data.clip_start = 0.1
obj_camera.data.clip_end = 1000.0

# Increase render resolution
bpy.context.scene.render.resolution_x = 3840
bpy.context.scene.render.resolution_y = 2160
bpy.context.scene.render.resolution_percentage = 100

# Set the output format to PNG
bpy.context.scene.render.image_settings.file_format = 'PNG'

# Enable RGBA for transparency
bpy.context.scene.render.image_settings.color_mode = 'RGBA'

# Set transparency settings
bpy.context.scene.render.film_transparent = False

# render and save image
bpy.context.scene.render.image_settings.file_format = 'PNG'


# Set the render engine to Eevee
bpy.context.scene.render.engine = 'BLENDER_EEVEE'

# Enable bloom in the render settings
bpy.context.scene.eevee.use_bloom = True

# Adjust bloom settings
bpy.context.scene.eevee.bloom_threshold = 0.0
bpy.context.scene.eevee.bloom_knee = 0.0
bpy.context.scene.eevee.bloom_radius = 0.0
bpy.context.scene.eevee.bloom_color = (1.0, 1.0, 1.0)
bpy.context.scene.eevee.bloom_intensity = 0.5
bpy.context.scene.eevee.bloom_clamp = 0.0

bpy.context.scene.render.filepath = 'C:/Users/maxd5/Documents/GitHub/deformable-pov-display/scripts (python)/troposkein modeling/images/with_bloom.png'
bpy.ops.render.render(write_still = True)

# Enable bloom in the render settings
bpy.context.scene.eevee.use_bloom = False

bpy.context.scene.render.filepath = 'C:/Users/maxd5/Documents/GitHub/deformable-pov-display/scripts (python)/troposkein modeling/images/bwithout_bloom.png'
bpy.ops.render.render(write_still = True)


print( f'Total time = {time.time() - t} seconds' )
