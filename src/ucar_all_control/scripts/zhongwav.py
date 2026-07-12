#! /home/ucar/miniforge3/envs/opencv_env/bin/python
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
import os

audio_dict = {
            #水果组终点播报
            "Apple_Apple":"/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/apple_apple_x.wav",
            "Banana_Banana": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/banana_banana_x.wav",
            "Watermelon_Watermelon": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/watermelon_watermelon_x.wav",
            "Apple_Banana": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/apple_banana_x.wav",
            "Apple_Watermelon": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/apple_watermelon_x.wav",
            "Banana_Watermelon": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/banana_watermelon_x.wav",
            #蔬菜组终点播报
            "Chili_Chili": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/chili_chili_x.wav",
            "Tomato_Tomato": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/tomato_tomato_x.wav",
            "Potato_Potato": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/potato_potato_x.wav",
            "Chili_Tomato": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/chili_tomato_x.wav",
            "Chili_Potato": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/chili_potato_x.wav",
            "Tomato_Potato": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/tomato_potato_x.wav",
            #甜品组终点播报
            "Milk_Milk": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/milk_milk_x.wav",
            "Cake_Cake": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/cake_cake_x.wav",
            "Cola_Cola": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/cola_cola_x.wav",
            "Milk_Cake": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/milk_cake_x.wav",
            "Milk_Cola": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/milk_cola_x.wav",
            "Cake_Cola": "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/cake_cola_x.wav"
        }


def final_output(thing1,thing2):
    # 播放音频（如果在字典中）
    if f"{thing1}_{thing2}" in audio_dict:
        audio_file = audio_dict[f"{thing1}_{thing2}"]
        os.system(f"aplay {audio_file}")
    elif f"{thing2}_{thing1}" in audio_dict:
        audio_file = audio_dict[f"{thing2}_{thing1}"]
        os.system(f"aplay {audio_file}")
    else:
        rospy.loginfo("无")



if __name__ == "__main__":
    rospy.init_node("zhongwav")
    # rospy.set_param("/thing1",0)
    # rospy.set_param("/thing2",0)
    flag=rospy.get_param("/all_done")
    while flag!=1:
        flag=rospy.get_param("/all_done")
    rospy.loginfo("開始終點播报")
    thing1=rospy.get_param("/thing1")
    thing2=rospy.get_param("/thing2")
    final_output(thing1,thing2)
    rospy.spin()
     