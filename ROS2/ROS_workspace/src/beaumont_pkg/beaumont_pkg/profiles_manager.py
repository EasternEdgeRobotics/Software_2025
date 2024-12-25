from eer_messages.srv import Config
import os

import rclpy
from rclpy.node import Node
import json


def delProfile(profileDel, file):
        try: 
            with open(file, "r") as f:
                profiles  = json.loads(f.read()) # reads profiles from file
                
                for i, profile in enumerate(profiles):
                    if profile["profileName"] == profileDel: # delete entry of the profile 
                        profiles.pop(i) 
                            
                        return "Profile Deleted", profiles
        
        except FileNotFoundError:
            return "Profile Not Found", []


class ProfilesManager(Node):

    def __init__(self):
        super().__init__('profiles_manager')

        #The ROS services below communicate straight with the GUI, as long as rosbridge_server is running
        #The service names ("profile_config" and "profiles_list") MUST match those defined in the GUI
        self.srv1 = self.create_service(Config, "profile_config", self.profile_config_callback)
        self.srv2 = self.create_service(Config, "profiles_list", self.profiles_list_callback)
        self.srv3 = self.create_service(Config, "camera_urls", self.camera_urls_callback)

    

    def profile_config_callback(self, request, response):
        ''' 
        The structure of how the data is stored is:
        profileName:

            controller 1:
                controllerName
            0: 
                buttons
                    number : actions
                axes
                    number : actions
                deadzones
                    number : value

            controller 2:
                controllerName
            1: 
                buttons
                    number : actions
                axes
                    number : actions
                deadzones
                    number : value

            these are stored as part of a JSON file 
            '''

            #######################
            # Here's Exactly what you should expect to recieve:
            '''
            {"profileName": "Zaid", "controller1": "Xbox One Game Controller (STANDARD GAMEPAD)", "controller2": "null", "associated_mappings": {"0": {"buttons": {"0": "None", "1": "None", "2": "None", "3": "None", "4": "heave_up", "5": "open_claw", "6": "heave_down", "7": "close_claw", "8": "None", "9": "None", "10": "None", "11": "None", "12": "None", "13": "None", "14": "None", "15": "None", "16": "None"}, "axes": {"0": "sway", "1": "surge", "2": "yaw", "3": "pitch"}, "deadzones": {"0": "0", "1": "0", "2": "0", "3": "0"}}, "1": {}}}
            '''
            #######################

            newMappings  = json.loads(request.data)

            current_directory = os.path.dirname(os.path.abspath(__file__)) # create directory if not done previously
            FILE_PATH = f"{current_directory}/config/profiles.json"
            os.makedirs(os.path.dirname(FILE_PATH), exist_ok=True) 
             

            result, newProfiles = delProfile(newMappings["profileName"], FILE_PATH) # deletes previous profile with that name
            newProfiles.append(newMappings)

            with open(FILE_PATH, "w") as f:
                f.write(json.dumps(newProfiles)) # overwrites file with new profile list

                response.result = "success"
            
            return response                


        elif request.state == 1: #We are looking to load mapping into GUI from database
            current_directory = os.path.dirname(os.path.abspath(__file__))
            FILE_PATH = f"{current_directory}/config/profiles.json"
            os.makedirs(os.path.dirname(FILE_PATH), exist_ok=True)
        
            try:
                with open(FILE_PATH, "r") as f:
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

        current_directory = os.path.dirname(os.path.abspath(__file__))
        FILE_PATH = f"{current_directory}/config/profiles.json"
        os.makedirs(os.path.dirname(FILE_PATH), exist_ok=True)
        

        if request.state == 0: # The requested profile should be deleted

            response.result, newProfiles = Delprofile(request.data, FILE_PATH)

            with open(FILE_PATH, "w") as f:
                f.write(json.dumps(newProfiles)) #overwrites previous profiles with new list
            
            return response


        elif request.state == 1: #We only want to read the profiles

            profileNames = []

            try:
                with open(FILE_PATH, "r") as f:
                    profiles = json.loads(f.read())

                    for i, profile in enumerate(profiles): # write in the format expected by the client
                        profileNames.append({"id": i, 
                                            "name": profile["profileName"], 
                                            "controller1" : profile["controller1"], 
                                            "controller2" : profile["controller2"]})

            except FileNotFoundError:
                pass
            
            response.result = json.dumps(profileNames)

            return response
            

    def camera_urls_callback(self, request, response):

        if request.state == 0:  # We are looking to load camera URLs into database from GUI

            urls = json.loads(request.data)  # Turn the received string into a List

            while True:
                if len(urls) < 4:
                    urls.append("http://")
                else:
                    break

            current_directory = os.path.dirname(os.path.abspath(__file__))
            os.makedirs(os.path.dirname(f"{current_directory}/config/profiles.json"), exist_ok=True)

            with open(f"{current_directory}/config/camera_urls.json", "w") as f:
                f.write(json.dumps(urls))

            response.result = "Success"

            return response

        elif request.state == 1: # We are looking to fetch camera URLs form database into GUI

            outgoing_camera_urls = []

            current_directory = os.path.dirname(os.path.abspath(__file__))

            try:
                with open(f"{current_directory}/config/camera_urls.json", "r") as f:
                    urls = json.loads(f.read())
            except FileNotFoundError:
                urls = ["http://", "http://", "http://", "http://"]

            response.result = json.dumps(urls)

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
