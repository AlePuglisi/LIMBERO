# Python script to compute links inertia from material density and mesh file
import trimesh
import numpy as np
from ament_index_python.packages import get_package_share_directory


# Compute TITAN 4 Inertia: 
density = 1.25  #[g/cm^3], Titanium

links = ["gripper_Link_Hand", "gripper_Link_finger1", "gripper_Link_finger2", "gripper_Link_finger3"]
#arm_CoM_links = ["arm_base_CoM", "arm_link1_CoM", "arm_link2_CoM", "arm_link3_CoM", "arm_link4_CoM", "arm_link5_CoM", "grip_claw_CoM", "grip_claw_CoM"]
volume = []
mass = []
CoM = []
inertia_matrix = []
#inertia_matrix_CoM = []

i = 0
for link in links:
    mesh_path = get_package_share_directory('lbr_description')
    stl_file = mesh_path + "/meshes/Grieel_gripper_mode/CoM_Centered/" + link + ".stl"
    #stl_CoM_file = mesh_path + "/meshes/CoM_centered/" + link + "_CoM.stl"
    mesh = trimesh.load(stl_file)
    #mesh_CoM = trimesh.load(stl_CoM_file)
    volume.append(mesh.volume)              # [m^2]
    mass.append(mesh.volume*(density*1000)) # [Kg]
    CoM.append(mesh.center_mass)            # [m] (1x3)
    # inertia_matrix.append(mesh.moment_inertia) # [(Kg*m^2)/(kg*m^3)] (3x3) Normalized Inertia
    inertia_matrix.append(mesh.moment_inertia*mass[i]/volume[i]) # (3x3) [Kg*m^2] 
    #inertia_matrix_CoM.append(mesh_CoM.moment_inertia*mass[i]/volume[i])

    print(f"\nLink {link}: \n Volume= {volume[i]}\nMass= {mass[i]}\nCenter of Mass= {CoM[i]}\nInertia Matrix Origin= {inertia_matrix[i]}\n------------------------")#Inertia Matrix CoM= {inertia_matrix_CoM[i]}\n------------------")
    i+=1

print("\ntotal mass: " + str(sum(mass)))

