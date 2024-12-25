from eer_messages.srv import Config, Cameras
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
        ''' 
        The structure of how the data is stored is:
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
        
        # Create config folder if not done previously
        current_directory = os.path.dirname(os.path.abspath(__file__)) 
        file_path = f"{current_directory}/config/profiles.json"
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        if request.state == 0: #We are looking to load mappings into database from GUI

            newMappings  = json.loads(request.data)

            newProfiles, result = delProfile(newMappings["profileName"], file_path) # deletes previous profile with that name
            newProfiles.append(newMappings)

            with open(file_path, "w") as f:
                f.write(json.dumps(newProfiles)) # overwrites file with new profile list

            response.result = "success"
            
            return response                


        elif request.state == 1: #We are looking to load mapping into GUI from database
            try:
                with open(file_path, "r") as f:
                    profiles  = json.loads(f.read())

                    for profile in profiles:
                        if profile["profileName"] == request.data:
                            response.result = json.dumps(profile["associated_mappings"]) 
                            return response # only the associated mappings for that profile are returned

            except FileNotFoundError:
                pass
                
            defaultmap = {"buttons": {}, "axes": {}, "deadzones": {}}
            response.result = json.dumps({0: defaultmap, 1: defaultmap}) 

            return response


    def profiles_list_callback(self, request, response):

        # Create config folder if not done previously
        current_directory = os.path.dirname(os.path.abspath(__file__))
        file_path = f"{current_directory}/config/profiles.json"
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        

        if request.state == 0: # The requested profile should be deleted

            newProfiles, response.result = Delprofile(request.data, file_path)

            with open(file_path, "w") as f:
                f.write(json.dumps(newProfiles)) #overwrites previous profiles with new list
            
            return response


        elif request.state == 1: #We only want to read the profiles

            profileNames = []

            try:
                with open(file_path, "r") as f:
                    profiles = json.loads(f.read())

                    for index, profile in enumerate(profiles): # write in the format expected by the client
                        profileNames.append({"id": index, 
                                            "name": profile["profileName"], 
                                            "controller1" : profile["controller1"], 
                                            "controller2" : profile["controller2"]})

            except FileNotFoundError:
                pass
            
            response.result = json.dumps(profileNames)

            return response
            

    def camera_urls_callback(self, request, response):

        # Create config folder if not done previously
        current_directory = os.path.dirname(os.path.abspath(__file__))
        file_path = f"{current_directory}/config/camera_urls.json"
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

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

            with open(file_path, "w") as f:
                f.write(json.dumps(urls))

            response.success = True

            return response

        elif request.state == 1: # We are looking to fetch camera URLs form database into GUI

            outgoing_camera_urls = []

            try:
                with open(file_path, "r") as f:
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
