#! /usr/bin/env python
#The above tells Linux that this file is a Python script,
#and that the ROS should use the Python interpreter in
#/usr/bin/env
#to run it. using "chmod +x [filename]" to make
#this script executable is very important if the rosrun wants to
#be used under ROS.
##############################################################
##              University of Gavle, Sweden                 ##
##----------------------------------------------------------##
##      EE470D | Master Thesis in Electronics/Automation    ##
##----------------------------------------------------------##
##                                                          ##
##     Development of Python Algorithms to execute holding, ##
##      lifting and Positioning task by Baxter Robot        ##
##                                                          ##
##                                                          ##
##----------------------------------------------------------##
##                  Student: Rabe Andersson                 ##
##############################################################
##                                                          ##
##                          2019                            ##
##  This is a program of is used to move objects that is    ##
##  arranged in two arrays (3*4) to (4*3) by using Baxter   ##
##  robot that shows the picking, holding and placing       ##
##  objects in different locations                          ##
##                                                          ##
##############################################################
##############################################################

import rospy
#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.

import baxter_interface
#Import the baxter_interface Library.

from baxter_interface import CHECK_VERSION
#Import from the baxter_interface the check_version as well.
#Run this program as a new node in the ROS computation graph
#called /Pick_arm to execute Picking, holding and positioning the object

rospy.init_node("Pick_arm")
print("Getting robot state... ")

rs = baxter_interface.RobotEnable(CHECK_VERSION)
#To enable the robot in the beginning of the python script and before 
#achieving the task

init_state = rs.state().enabled
print("Enabling robot before starting the action... ")
rs.enable()

h = baxter_interface.Head()
r = baxter_interface.Limb("right")
rightgripper = baxter_interface.Gripper('right')
r.move_to_neutral()

P ={};PUp = {};T = {};TUp = {}
# All the positions that Baxter robot will move to and from in 
# both building the leavingand pickingthecubes from the certainplaces

P[1] = {'right_s0': 0.10431069357620813, 'right_s1': 0.6059224112147384, 'right_w0': -2.196276993054941, 'right_w1': 1.291228328202547, 'right_w2': 0.1169660350762628, 'right_e0': 1.7867041226895357, 'right_e1': 0.8256651590793239}
PUp[1] = {'right_s0': 0.0011504855909140602,'right_s1': 0.46901462589596526, 'right_w0': -2.1445051414638083, 'right_w1': 1.647111870991963, 'right_w2': 0.2998932440315984, 'right_e0': 1.896767244220314, 'right_e1': 1.1681263699747426}

P[2] = {'right_s0': -0.07938350577307016, 'right_s1': 0.744364177321397, 'right_w0': -2.386107115555761, 'right_w1': 1.4342720366728618, 'right_w2':0.2511893540162365, 'right_e0': 1.7537235357499992, 'right_e1': 1.1608399612322868}
PUp[2] = {'right_s0': -0.10699515995500761, 'right_s1': 0.5752427954570302, 'right_w0': -2.1847721371458007, 'right_w1': 1.7767332475682804, 'right_w2': 0.33287383097113477, 'right_e0': 1.912874042493111, 'right_e1': 1.3694613483847031}

P[3] = {'right_s0': -0.3344078117590202, 'right_s1': 0.7370777685789413, 'right_w0': -2.3669323557071933, 'right_w1': 1.4143302864303515, 'right_w2': 0.3877136441380383, 'right_e0': 1.5010002009458774, 'right_e1': 1.4607332052638853}
PUp[3] = {'right_s0': -0.28532042654668693, 'right_s1':0.6247136758663347, 'right_w0': -2.1824711659639724, 'right_w1': 1.8150827672654157, 'right_w2': 0.41647578391088985, 'right_e0': 1.7928400458410774, 'right_e1': 1.6497963373707625}

P[4] = {'right_s0': -0.4931748233051605, 'right_s1': 0.7110000951848893,'right_w0': -2.318611960888803, 'right_w1': 1.4304370847031482, 'right_w2': 0.44178646691099915, 'right_e0': 1.358339987672534, 'right_e1': 1.6164322552342547}
PUp[4] = {'right_s0': -0.44677190447162674, 'right_s1': 0.6408204741391316, 'right_w0': -2.1736507764336315, 'right_w1': 1.8342575271139834, 'right_w2': 0.5100486119719001, 'right_e0': 1.6643691548556738, 'right_e1': 1.8699225804323194}

P[5] = {'right_s0': 0.0337475773334791, 'right_s1': 0.5948010505025692, 'right_w0': -2.1778692236003163, 'right_w1': 1.2501943421266122, 'right_w2': -0.006902913545484362, 'right_e0': 1.804344901750218, 'right_e1': 0.7719758315033345}
PUp[5] = {'right_s0': 0.02454369260616662, 'right_s1': 0.5414952181235511, 'right_w0': -2.3128595329342327, 'right_w1': 1.7077041121134369, 'right_w2': 0.01457281748491143, 'right_e0': 2.118043972872785, 'right_e1': 1.02853411827717}

P[6] = {'right_s0': -0.2281796421979553, 'right_s1': 0.6872233929726653, 'right_w0': -2.263388652524928, 'right_w1': 1.3192234775814558, 'right_w2': 0.13192234775814557, 'right_e0': 1.674340029976929, 'right_e1': 1.1558545236716593}
PUp[6] = {'right_s0': -0.15569904997036949, 'right_s1': 0.6661311571392409, 'right_w0': -2.3151605041160606, 'right_w1': 1.8016604353714185,'right_w2': 0.09088836168221076, 'right_e0': 2.0716410540392514, 'right_e1': 1.3341797902633385}

P[7] = {'right_s0': -0.3324903357741634, 'right_s1': 0.7915340865488735, 'right_w0': -2.4037478946164432, 'right_w1': 1.4856603930670231, 'right_w2': 0.19673303604630432, 'right_e0': 1.6674371164314448, 'right_e1': 1.3219079439602552}
PUp[7] = {'right_s0': -0.3903981105168378, 'right_s1': 0.6273981422451342, 'right_w0': -2.20164592581254, 'right_w1': 1.7702138292197676, 'right_w2': 0.3551165523954733, 'right_e0': 1.806645872932046, 'right_e1': 1.6375244910676792}

P[8] = {'right_s0': -0.5595194923812047, 'right_s1': 0.6845389265938658, 'right_w0': -2.2921507922977793, 'right_w1': 1.405126401703039, 'right_w2': 0.31676703269833795, 'right_e0': 1.3778982427180728, 'right_e1': 1.5619759372643225}
PUp[8] = {'right_s0': -0.5196359918961839, 'right_s1': 0.5925000793207411, 'right_w0': -2.13645174232741, 'right_w1': 1.808946844113874, 'right_w2': 0.40420393760780654, 'right_e0': 1.6846944002951556, 'right_e1': 1.7916895602501632}

P[9] = {'right_s0': 0.05368932757598948, 'right_s1': 0.47169909227476475, 'right_w0': -1.9209274416295095, 'right_w1': 1.171961321944456, 'right_w2': -0.19711653124327566, 'right_e0': 1.6505633277647052, 'right_e1': 0.5503156076538922}
PUp[9] = {'right_s0': -0.0674951546669582, 'right_s1': 0.41724277430483253, 'right_w0': -2.1675148532820896, 'right_w1': 1.6260196351585385, 'right_w2': 0.01457281748491143, 'right_e0': 1.9512235621902463, 'right_e1': 0.9759952762920945}

P[10] = {'right_s0': -0.19903400722813244, 'right_s1': 0.6239466854723921, 'right_w0': -2.190524565100371, 'right_w1': 1.3119370688390002, 'right_w2': -0.004218447166684887, 'right_e0': 1.7103885784922364, 'right_e1': 0.9744612955042091}
PUp[10] = {'right_s0': -0.26537867630417655, 'right_s1': 0.4183932598957466, 'right_w0': -2.033291534342116, 'right_w1': 1.6517138133556193, 'right_w2': 0.15186409800065595, 'right_e0': 1.8093303393108455, 'right_e1': 1.240223467005357}

P[11] = {'right_s0': -0.45022336124436896, 'right_s1': 0.6377525125633607, 'right_w0': -2.1886070891155143, 'right_w1': 1.3092526024602007, 'right_w2': 0.10891263593986437, 'right_e0': 1.5251603983550726, 'right_e1': 1.2843254146570626}
PUp[11] = {'right_s0': -0.47821851062327775, 'right_s1': 0.41647578391088985, 'right_w0': -1.9960925002358947, 'right_w1': 1.6041604089311714, 'right_w2': 0.25617479157686407, 'right_e0': 1.6390584718555645, 'right_e1': 1.5500875861582106}

P[12] = {'right_s0': -0.5882816321540562, 'right_s1': 0.68568941218478, 'right_w0': -2.2952187538735505, 'right_w1': 1.4200827143849217, 'right_w2': 0.13767477571271589, 'right_e0': 1.4511458253396015, 'right_e1': 1.463801166839656}
PUp[12] = {'right_s0': -0.6308495990178764, 'right_s1': 0.38426218736529616, 'right_w0': -1.9773012355842983, 'right_w1': 1.6106798272796845, 'right_w2': 0.3021942152134265, 'right_e0': 1.5374322446581559, 'right_e1': 1.7575584877197128}

T[1] = {'right_s0': 0.6258641614572488, 'right_s1': 0.5445631796993219, 'right_w0': -2.081228433963535, 'right_w1': 1.2252671543234743, 'right_w2': 0.6991117440787773,'right_e0': 1.6459613854010489, 'right_e1': 0.870150601928001}
TUp[1] = {'right_s0': 0.5250049246537829, 'right_s1': 0.2577087723647495, 'right_w0': -1.8438449070382674, 'right_w1': 1.4894953450367368, 'right_w2': 0.9085001216251363, 'right_e0': 1.6168157504312262, 'right_e1': 1.1688933603686853}

T[2] = {'right_s0': 0.45520879880499654, 'right_s1': 0.49432530889607457, 'right_w0': -2.0210196880390328, 'right_w1': 1.1922865673839378, 'right_w2': 0.7485826244880819, 'right_e0': 1.4871943738549087, 'right_e1': 1.026616642292313}
TUp[2] ={'right_s0': 0.426446659032145, 'right_s1': 0.21015536794030168, 'right_w0': -1.7518060597651426, 'right_w1': 1.469170099597255, 'right_w2': 0.7600874803972225, 'right_e0': 1.5428011774157548, 'right_e1': 1.190369091399081}

T[3] = {'right_s0': 0.32098547986502285, 'right_s1': 0.6189612479117644, 'right_w0': -2.184388641948829, 'right_w1': 1.2881603666267762, 'right_w2': 0.6258641614572488, 'right_e0': 1.6336895390979655, 'right_e1': 1.0845244170349875}
TUp[3] ={'right_s0': 0.273432075440575, 'right_s1': 0.35319907641061654, 'right_w0': -1.9266798695840797, 'right_w1': 1.6010924473554007, 'right_w2': 0.6703496043059258, 'right_e0': 1.6954322658103536, 'right_e1': 1.313087554429914}

T[4] ={'right_s0': 0.18676216092504913, 'right_s1': 0.7136845615636888, 'right_w0': -2.298286715449321, 'right_w1': 1.3391652278239663, 'right_w2': 0.5104321071688714, 'right_e0': 1.718825472825606, 'right_e1': 1.1504855909140603}
TUp[4] ={'right_s0': 0.14879613642488512, 'right_s1': 0.37160684586524145, 'right_w0': -1.9351167639174494, 'right_w1': 1.6068448753099709, 'right_w2': 0.6622962051695274, 'right_e0': 1.6954322658103536, 'right_e1': 1.3571895020816198}

T[5] = {'right_s0': 0.5100486119719001, 'right_s1': 0.6139758103511368, 'right_w0': -2.169432329266946, 'right_w1': 1.3176894967935704, 'right_w2': 0.8736020587007431, 'right_e0': 1.6007089521584292, 'right_e1': 1.1362962686261202}
TUp[5] ={'right_s0': 0.431815591789744, 'right_s1': 0.2527233348041219, 'right_w0': -1.8714565612202048, 'right_w1': 1.540883701430898, 'right_w2': 1.0921943209744145, 'right_e0': 1.5734807931734631, 'right_e1': 1.4741555371578825}

T[6] = {'right_s0': 0.3474466484560462, 'right_s1': 0.49125734732030374, 'right_w0': -2.0678061020695377, 'right_w1': 1.2237331735355887, 'right_w2': 0.9326603190343316, 'right_e0': 1.3729128051574453, 'right_e1':1.2927623089904325}
#{'right_s0': 0.3926990816986659, 'right_s1': 0.4939418136991032, 'right_w0': -1.9995439570086369, 'right_w1': 1.2659176452024377, 'right_w2': 0.8551942892461182, 'right_e0': 1.419315723990979, 'right_e1': 1.1930535577778805}
TUp[6] ={'right_s0': 0.2784175130012026, 'right_s1': 0.3282718886074785, 'right_w0': -1.8756750083868896, 'right_w1': 1.5589079756885518, 'right_w2': 0.9821311994436361, 'right_e0': 1.6118303128705984, 'right_e1': 1.5044516577186196}

T[7] = {'right_s0': 0.17985924737956477, 'right_s1': 0.674951546669582, 'right_w0': -2.2407624359036182, 'right_w1': 1.3568060068846484, 'right_w2': 0.7420632061395689, 'right_e0': 1.5366652542642132, 'right_e1': 1.3330293046724246}
TUp[7] ={'right_s0': 0.17717478100076528, 'right_s1':0.32903887900142126, 'right_w0': -1.85880121972015, 'right_w1': 1.6379079862646506, 'right_w2': 0.8797379818522848, 'right_e0': 1.631005072719166, 'right_e1': 1.5374322446581559}

T[8] ={'right_s0': 0.033364082136507746, 'right_s1': 0.7723593267003058, 'right_w0': -2.37153429807085, 'right_w1': 1.4112623248545806, 'right_w2': 0.6116748391693088, 'right_e0': 1.5853691442795752, 'right_e1': 1.397456497763612}
TUp[8] ={'right_s0': 0.0724805922275858, 'right_s1': 0.3696893698803847, 'right_w0': -1.9213109368264807, 'right_w1': 1.6517138133556193, 'right_w2': 0.8084078752156131, 'right_e0': 1.631005072719166, 'right_e1': 1.5642769084461507}

T[9] = {'right_s0': 0.3762087882288977, 'right_s1': 0.7478156340941392, 'right_w0': -2.364247889328394, 'right_w1': 1.4089613536727525, 'right_w2': 1.00590790165586, 'right_e0': 1.5454856437945543, 'right_e1': 1.4254516471425207}
TUp[9] ={'right_s0': 0.3566505331833587, 'right_s1': 0.28608741694062967, 'right_w0': -1.9063546241445979, 'right_w1': 1.5401167110369554, 'right_w2': 1.2482768661417554, 'right_e0': 1.527844864733872, 'right_e1': 1.7119225592801217}

T[10] = {'right_s0': 0.22702915660704123, 'right_s1': 0.6392864933512462, 'right_w0': -2.2422964166915036, 'right_w1': 1.3541215405058489, 'right_w2': 0.9779127522769513, 'right_e0': 1.402825430521211,'right_e1': 1.496398258582221}
TUp[10] ={'right_s0': 0.24505343086469483, 'right_s1': 0.3025777104103979, 'right_w0': -1.8691555900383767, 'right_w1': 1.5934225434159734, 'right_w2': 1.2053254040809638, 'right_e0': 1.5266943791429581, 'right_e1': 1.7326312999165747}

T[11] = {'right_s0': 0.08206797215186963, 'right_s1': 0.7002622296696914, 'right_w0': -2.2618546717370425, 'right_w1': 1.41394679123338, 'right_w2': 0.8394709861702927, 'right_e0': 1.4335050462789192, 'right_e1': 1.5109710760671324}
TUp[11] ={'right_s0': 0.09050486648523941, 'right_s1': 0.49585928968396, 'right_w0': -2.022553668826918, 'right_w1': 1.7433691654317727, 'right_w2': 1.0569127628530501, 'right_e0': 1.6428934238252781, 'right_e1': 1.7985924737956476}

T[12] = {'right_s0': -0.07363107781849985, 'right_s1': 0.7171360183364309, 'right_w0': -2.292917782691722, 'right_w1': 1.4281361135213202, 'right_w2': 0.7608544707911652, 'right_e0': 1.4089613536727525, 'right_e1': 1.5585244804915803}
TUp[12] ={'right_s0':-0.04793689962141918, 'right_s1': 0.3911651009107805, 'right_w0': -1.945471134235676, 'right_w1': 1.6616846884768743, 'right_w2': 0.9246069198979331, 'right_e0': 1.5481701101733538, 'right_e1': 1.8265876231745564}

iter= 0

# To ensure that the Right gripper will be open before starting the action
rightgripper.open()
rospy.sleep(0.5)

# Moving the head to the right and left which imitating the human behavior in examining the environment.
#h.set_pan(-1, speed=0.20, timeout=10)  #moving the head to the right with the speed of 20% of the full allowed velocity
#h.set_pan(1, speed=0.20, timeout=10) #moving the head to the left with the speed of 20% of the full allowed velocity

#To let the robot seems to look ahead towards the cubeswith the speed 0.20
#h.set_pan(0, speed=0.20, timeout=10)

# Argument that gives the user to choose between two alternatives: 
# (A)automatic and (M)manual:
slc = raw_input('Do you want to run (a)automatic or (m)manual. Choose either a or m ')

# The loop while to do the first phase of theprogram(Moving the cubes
# from the array(4*3) to the other location and building another array (3*4))
# #Keep publishing until a Ctrl-C is not pressed
while not rospy.is_shutdown():
    
    # move from Ps(array(4*3)) to Ts(array(3*4))
    # if the manual(m) is chosen so enter this manual phase
    if slc == "m":
        raw_input("Ready to make a move the cubes and the magazine is loaded, if so... Press Enter to continue...")
        print("Performing the cubemoving action, Any time Ctrl+Cquits the program")
        
        #This is a for loop for 12 cubes 
        for i in range(1,13):

        # To Move the head towards the object that the robot will pick.
            #h.set_pan(-0.25, speed=0.20, timeout=10)
        
            # Moving the arm to the position which is above the object.
            ## go to Pup 
            r.move_to_joint_positions(PUp[i])
            print("PUp"+ str(i) + "th UP location")
            rospy.sleep(0.5)
            # Moving the arm to down to the object.
            ## go to P
            r.move_to_joint_positions(P[i])
            print("P"+ str(i) + "th location")
            rospy.sleep(0.5)

            # Closing the gripper which means grasp the object if it is the 
            # gripper is used in the robot and suck the object and holding 
            # it if the vacuum cup gribber is used.
            # close gripper
            rightgripper.close()
            rospy.sleep(0.5)
            print("gripper closed")
            
            # Moving the arm up to the position which is above the object.
            # go to Pup
            r.move_to_joint_positions(PUp[i])
            rightgripper.close()
            rospy.sleep(0.5)
            print("PUp"+ str(i) + "th UP location")
            rospy.sleep(0.5)
            
            # To Move the head towards the new location that the robot will 
            # place the cubes
            #h.set_pan(0.25, speed=0.20, timeout=10)
            
            # Moving the arm towards the TUp positions
            r.move_to_joint_positions(TUp[i])
            print("TUp"+ str(i) + "thlocation")
            rospy.sleep(0.5)
            
            # moving the arm towards the T position which means the new 
            # locations that the cubeswill be places.
            # go to T
            r.move_to_joint_positions(T[i])
            print("T"+ str(i) + "th location")
            rospy.sleep(0.5)
            
            # open gripper
            rightgripper.open()
            print("gripper opened and Nr. of cubesthat are left to the new place or array(3*4):"
                    +str(i))
            rospy.sleep(0.5)
            
            #Moving to the TUp positions in order to move vertically 
            r.move_to_joint_positions(TUp[i])
            print("TUp"+ str(i) + "thlocation")
            rospy.sleep(0.5)
        
