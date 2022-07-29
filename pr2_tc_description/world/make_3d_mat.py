import random as rand
import math
import numpy as np

'''
Define global_variables
'''

map = "map_array_res08.txt"      # Obstacle array for map, 1->free, 0->obstacle
boxes = 3000

def make_obs_list(file_name):
    """
    Create list of obstacles given input height_map of obstacle heights
    :param obs_list: list of obstacles where each obstacle defined as [p1, p2, p3, p4] where p = [x,y,z]
    :return: obs_list
    """
    ################# Convert input file to 2D matrix of block heights
    map_array = []

    try:
        map_array = np.loadtxt(file_name, delimiter=",")

    except IOError:
        print('Error: file name provided to make_3d_mat does not exist')
        return []

    # Init obs list
    obs_list = []
    start_box = False
    size_row = 1
    for row in range(len(map_array)):
        for col in range(len(map_array[0])):

            # Define the row and col corresponding to the obstacle spaces
            if map_array[row][col] == 0 and start_box == False: #this means we have an obstacle here

                #p_start_row = row
                #print(p_start_row)
                p_start_col = col
                #print(p_start_col)
                start_box = True
            
            if col+1 < len(map_array[0]) and map_array[row][col+1] == 1 and start_box == True: #box has ended                
                size_col = (col-1)-p_start_col
                p1 = row+0.5 #row center of box
                p2 = p_start_col+(size_col/2) #col center of box

                obs_list.append([p1, p2, size_row, size_col])
                start_box = False
    return obs_list


def make_obs_models(obs_list, file_name):
    """
    Create .txt file containing obstacle models defined by obs_list, in Gazebo model format
    :param obs_list: list of obstacles
    :param file_name: output .txt file
    :return:
    """

    # Write to .txt file. If it already exists, display error
    try:
        f_check = open(file_name, "r")
        f_check.close()
        print('Error: .world file already exists. Change filename.')
    except IOError:
        f_new = open(file_name, "w+")

        # Add spacing in between
        f_new.write("<sdf version='1.6'> \n")
        f_new.write("  <world name='default'> \n")
        
        f_new.write("    <include> \n")
        f_new.write("      <uri>model://ground_plane</uri> \n")
        f_new.write("    </include> \n")
        f_new.write("      <collision name='collision'> \n")
        f_new.write("         <max_contacts>0</max_contacts> \n")
        f_new.write("      </collision> \n")
        f_new.write("    <light name='sun' type='directional'> \n")
        f_new.write("      <cast_shadows>0</cast_shadows> \n")
        f_new.write("      <pose frame=''>0 0 10 0 -0 0</pose> \n")
        f_new.write("      <diffuse>0.8 0.8 0.8 1</diffuse> \n")
        f_new.write("      <specular>0.2 0.2 0.2 1</specular> \n")
        f_new.write("      <attenuation> \n")
        f_new.write("        <range>1000</range> \n")
        f_new.write("        <constant>0.9</constant> \n")
        f_new.write("        <linear>0.01</linear> \n")
        f_new.write("        <quadratic>0.001</quadratic> \n")
        f_new.write("      </attenuation> \n")
        f_new.write("      <direction>-0.5 0.1 -0.9</direction> \n")
        f_new.write("    </light> \n")
        f_new.write("    <physics name='default_physics' default='0' type='ode'> \n")
        f_new.write("      <max_step_size>0.002</max_step_size> \n")
        f_new.write("    </physics> \n")
        f_new.write("    <gravity>0 0 -9.8</gravity> \n")
        f_new.write(" \n")
        f_new.write(" \n")

        # Add model def within main section of world
        for i in range(len(obs_list)):
        #for i in range(boxes):
            obs_cen_x = obs_list[i][0] 
            obs_cen_y = obs_list[i][1]
            obs_cen_z = float(3/2)
            height = 3
            
            f_new.write("    <model name='unit_box_%03i'>\n" % i)
            f_new.write("        <pose frame=''>%8.2f %8.2f %8.2f 0 0 0</pose>\n" % (obs_cen_x, obs_cen_y, obs_cen_z))
            f_new.write("      <static>1</static>\n")
            f_new.write("      <link name='link'>\n")
            f_new.write("        <inertial>\n")
            f_new.write("          <mass>5000</mass>\n")
            f_new.write("          <inertia>\n")
            f_new.write("            <ixx>200</ixx>\n")
            f_new.write("            <ixy>0</ixy>\n")
            f_new.write("            <ixz>0</ixz>\n")
            f_new.write("            <iyy>200</iyy>\n")
            f_new.write("            <iyz>0</iyz>\n")
            f_new.write("            <izz>200</izz>\n")
            f_new.write("          </inertia>\n")
            f_new.write("        </inertial>\n")
            f_new.write("        <collision name='collision'>\n")
            f_new.write("          <geometry>\n")
            f_new.write("            <box>\n")
            f_new.write("              <size>1 1 1</size>\n")
            f_new.write("            </box>\n")
            f_new.write("          </geometry>\n")
            f_new.write("          <max_contacts>10</max_contacts>\n")
            f_new.write("          <surface>\n")
            f_new.write("            <contact>\n")
            f_new.write("              <ode/>\n")
            f_new.write("            </contact>\n")
            f_new.write("            <bounce/>\n")
            f_new.write("            <friction>\n")
            f_new.write("              <torsional>\n")
            f_new.write("                <ode/>\n")
            f_new.write("              </torsional>\n")
            f_new.write("              <ode/>\n")
            f_new.write("            </friction>\n")
            f_new.write("          </surface>\n")
            f_new.write("        </collision>\n")
            f_new.write("        <visual name='visual'>\n")
            f_new.write("          <geometry>\n")
            f_new.write("            <box>\n")
            f_new.write("              <size>1 1 1</size>\n")
            f_new.write("            </box>\n")
            f_new.write("           </geometry>\n")
            f_new.write("          <material>\n")
            f_new.write("            <script>\n")
            f_new.write("              <name>Gazebo/Orange</name>\n")
            f_new.write("              <uri>file://media/materials/scripts/gazebo.material</uri>\n")
            f_new.write("            </script>\n")
            f_new.write("          </material>\n")
            f_new.write("        </visual>\n")
            f_new.write("      </link>\n")
            f_new.write("    </model>\n")

        f_new.write(" \n")
        f_new.write(" \n")
        f_new.write("    <state world_name='default'> \n")
        f_new.write("      <iterations>10</iterations> \n")
        f_new.write(" \n")
        f_new.write("      <model name='ground_plane'> \n")
        f_new.write("        <pose frame=''>0 0 0 0 -0 0</pose> \n")
        f_new.write("        <scale>1 1 1</scale> \n")
        f_new.write("        <link name='link'> \n")
        f_new.write("          <pose frame=''>0 0 0 0 -0 0</pose> \n")
        f_new.write("          <velocity>0 0 0 0 -0 0</velocity> \n")
        f_new.write("          <acceleration>0 0 0 0 0 0</acceleration> \n")
        f_new.write("          <wrench>0 0 0 0 -0 0</wrench> \n")
        f_new.write("        </link> \n")
        f_new.write("      </model> \n")
        f_new.write(" \n")
        f_new.write(" \n")
        
        # Add model def within main section of world
        for i in range(len(obs_list)):
        #for i in range(boxes):

            obs_cen_x = obs_list[i][0] 
            obs_cen_y = obs_list[i][1]
            obs_cen_z = float(3/2)
            height = 3
            if obs_list[i][3] != 0:

                f_new.write("      <model name='unit_box_%03i'>\n" % i)
            #f_new.write("        <pose frame=''>%8.2f %8.2f %8.2f 0 0 0</pose>\n" % (obs_cen_x, obs_cen_y, obs_cen_z))
                f_new.write("        <scale>%i %i %i</scale>\n" % (obs_list[i][2], obs_list[i][3], height))
                f_new.write("        <link name='link'>\n")
                f_new.write("          <pose frame=''>%8.2f %8.2f %8.2f 0 0 0</pose>\n" % (obs_cen_x, obs_cen_y, obs_cen_z))
                f_new.write("          <velocity>0 0 0 0 -0 0</velocity>\n")
                f_new.write("          <acceleration>0 0 0 0 -0 0</acceleration>\n")
                f_new.write("          <wrench>0 0 0 0 -0 0</wrench>\n")
                f_new.write("        </link>\n")
                f_new.write("      </model>\n")
            

        f_new.write(" \n")
        f_new.write(" \n")
        f_new.write("      <light name='sun'> \n")
        f_new.write("        <pose frame=''>0 0 10 0 -0 0</pose> \n")
        f_new.write("      </light> \n")
        f_new.write("    </state> \n")
        f_new.write("    <gui fullscreen='0'> \n")
        f_new.write("      <camera name='user_camera'> \n")
        f_new.write("        <pose frame=''>78.254211 34.307590 65.950531 0 1.002001 3.140001</pose> \n")
        f_new.write("        <view_controller>orbit</view_controller> \n")
        f_new.write("        <projection_type>perspective</projection_type> \n")
        f_new.write("      </camera> \n")
        f_new.write("    </gui> \n")
        f_new.write("  </world> \n")
        f_new.write("</sdf> \n")
        f_new.write(" \n")

        f_new.close()

def main():

    """
    Make new obstacle list
    """
    obs_list = make_obs_list(map)
    make_obs_models(obs_list,"map.world")

    """
    Init figure plot
    """

if __name__ == "__main__":
    main()

