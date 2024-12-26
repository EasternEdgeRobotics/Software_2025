#!/usr/bin/env python3
from eer_interfaces.srv import Config, Cameras
import os

import rclpy
from rclpy.node import Node
import json

def delProfile(profileDel, file_path):

    try:
        with open(file_path, "r") as f:
            profiles  = json.loads(f.read())
        
        # Find if the profile already exists and delete it
        previous_profiles = profiles

        profiles = [mapping for mapping in profiles if mapping["profileName"] != profileDel]

        if previous_profiles == profiles:
            return profiles, "Profile not found"
        else:
            return profiles, "Profile deleted"
                
    except FileNotFoundError:
        return [], "Profile not found"


class ProfilesManager(Node):

    def __init__(self):
        super().__init__('profiles_manager')

        #The ROS services below communicate straight with the GUI, as long as rosbridge_server is running
        #The service names ("profile_config" and "profiles_list") MUST match those defined in the GUI
        self.srv1 = self.create_service(Config, "profile_config", self.profile_config_callback)
        self.srv2 = self.create_service(Config, "profiles_list", self.profiles_list_callback)
        self.srv3 = self.create_service(Cameras, "camera_urls", self.camera_urls_callback)

    

    def profile_config_callback(self, request, response):
        if request.state == 0: #We are looking to load mappings into database from GUI

            ''' 
            The strucuture of how the data is stored is:
            profileName:

                controller 1:
                    controllerStatus
                0: 
                    buttons
                        actions
                    axes
                        actions
                        deadzones

                controller 2: (optional)
                    controllerStatus
                1: 
                    buttons
                        actions
                    axes
                        actions
                        deadzones

            these are stored as part of a JSON file 
            '''

            new_mapping  = json.loads(request.data)
            Pname = new_mapping["profileName"]

            profiles = []

            current_directory = os.path.dirname(os.path.abspath(__file__))

            # Make a config folder if it does not exist
            os.makedirs(os.path.dirname(f"{current_directory}/config/profiles.json"), exist_ok=True)

            profiles, _ = delProfile(Pname, f"{current_directory}/config/profiles.json")

            profiles.append(new_mapping)

            with open(f"{current_directory}/config/profiles.json", "w") as f:
                f.write(json.dumps(profiles)) # writes new profile in JSON format
            
            response.result = "success"
            
            return response          


        elif request.state == 1: #We are looking to load mapping into GUI from database

            profile_exists = False

            try:
                current_directory = os.path.dirname(os.path.abspath(__file__))
                os.makedirs(os.path.dirname(f"{current_directory}/config/profiles.json"), exist_ok=True)    

                with open(f"{current_directory}/config/profiles.json", "r") as f:
                    profiles  = json.loads(f.read())
                    
                    for mapping in profiles:
                        if mapping["profileName"] == request.data:
                            profile_exists = True
                            response.result = json.dumps(mapping["associated_mappings"])
            
            except FileNotFoundError:
                pass

            if not profile_exists:
                defaultmap = {"axes": {}, "buttons": {}, "deadzones": {}}
                response.result = json.dumps({0: defaultmap, 1: defaultmap}) #Turn the JSON object into a string
            
            return response


    def profiles_list_callback(self, request, response):

        current_directory = os.path.dirname(os.path.abspath(__file__))

        # Make a config folder if it does not exist
        os.makedirs(os.path.dirname(f"{current_directory}/config/profiles.json"), exist_ok=True)

        if request.state == 0: # The requested profile should be deleted

            profiles, response.result = delProfile(request.data, f"{current_directory}/config/profiles.json")

            with open(f"{current_directory}/config/profiles.json", "w") as f:
                f.write(json.dumps(profiles)) 

            return response

        elif request.state == 1: #We only want to read the profiles

            profilesNames = []

            try:
                with open(f"{current_directory}/config/profiles.json", "r") as f:
                    profiles = json.loads(f.read())
                    for index, mapping in enumerate(profiles):
                        profilesNames.append({"id": index, 
                                            "name":mapping["profileName"],
                                            "controller1": mapping["controller1"],
                                            "controller2": mapping["controller2"]})
            except FileNotFoundError:
                pass
            response.result = json.dumps(profilesNames)
            
            return response

    def camera_urls_callback(self, request, response):

        if request.state == 0:  # We are looking to load camera URLs into database from GUI

            # If camera urls are sent in json format, we need to turn them into a list
            if request.camera_urls == []:
                urls = json.loads(request.camera_urls_json)  # Turn the received string into a List
            else:
                urls = request.camera_urls

            # Ensure that there are 4 elements (4 cameras on Beaumont)
            while True:
                if len(urls) < 4:
                    urls.append("http://")
                else:
                    break

            os.makedirs("config", exist_ok=True)
            current_directory = os.path.dirname(os.path.abspath(__file__))

            if not os.path.exists(f"{current_directory}/config"):
                os.makedirs(f"{current_directory}/config")

            with open(f"{current_directory}/config/camera_urls.json", "w") as f:
                f.write(json.dumps(urls))

            response.success = True

            return response

        elif request.state == 1: # We are looking to fetch camera URLs form database into GUI

            outgoing_camera_urls = []

            current_directory = os.path.dirname(os.path.abspath(__file__))

            try:
                with open(f"{current_directory}/config/camera_urls.json", "r") as f:
                    urls = json.loads(f.read())
            except FileNotFoundError:
                urls = ["http://", "http://", "http://", "http://"]

            response.camera_urls = urls
            response.camera_urls_json = json.dumps(urls)

            return response

    
        

def main(args=None):
    # global session
    # Base.metadata.create_all(engine)
    # session = Session(engine)
    rclpy.init(args=args)
    profiles_manager = ProfilesManager()
    rclpy.spin(profiles_manager)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
