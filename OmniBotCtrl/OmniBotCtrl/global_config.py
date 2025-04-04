import os

ROOT_DIR = os.path.dirname(__file__)
ENVS_DIR = os.path.join(ROOT_DIR,'Env')
#Taitan Tinker Tinymal
ROBOT_SEL = 'Tinker'
#Trot Stand
GAIT_SEL = 'Trot'
PLAY_DIR ='/home/rx/Downloads/LocomotionWithNP3O-masteroldx/logs/rough_go2_constraint/Mar30_22-00-34_test_barlowtwins/model_30000.pt'   
#Sim2Sim Cmd
SPD_X = 0.3
SPD_Y = 0.0
SPD_YAW = 0


#train param
MAX_ITER = 30000
SAVE_DIV = 5000


#./compile XX.urdf XX.xml
#rosrun robot_state_publisher robot_state_publisher my_robot.urdf