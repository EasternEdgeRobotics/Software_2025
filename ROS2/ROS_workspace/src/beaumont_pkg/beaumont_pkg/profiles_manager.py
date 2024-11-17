from eer_messages.srv import Config
import os

import rclpy
from rclpy.node import Node
import json

#######################
# Add a new argument for file path
#######################
def delProfile(profileDel):
        #######################
        # use the file path that you passed in and open the file in read mode
        # Since you are using read mode, use a try except block to catch the FileNotFoundError
        # In the except block return an empty list if the file is not found along with the string "Profile Not Found"
        # In the try block, load the file and use json.loads() to get the list of profiles
        # Create a new list and iterate through the existing profiles. If the current mapping
        # in the exisitng profiles matches the condition mapping["profileName"] == profileDel, do not add it to the new list
        #######################
        with open("/app/src/beaumont_pkg/beaumont_pkg/config/profiles.json", "w") as f:
            profiles  = json.loads(f.read())
            if profileDel in profiles: 
                del profiles[profileDel] 
                        
                f.seek(0)
                f.write(json.dumps(profiles))
                f.truncate() # turns file to dict, deletes entry, writes at the beggining of the file, and eliminates past content
                return "Profile Deleted", f
            
            else: return "Profile Not Found", f


class ProfilesManager(Node):

    def __init__(self):
        super().__init__('profiles_manager')

        #The ROS services below communicate straight with the GUI, as long as rosbridge_server is running
        #The service names ("profile_config" and "profiles_list") MUST match those defined in the GUI
        self.srv1 = self.create_service(Config, "profile_config", self.profile_config_callback)
        self.srv2 = self.create_service(Config, "profiles_list", self.profiles_list_callback)
        self.srv3 = self.create_service(Config, "camera_urls", self.camera_urls_callback)

    

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

            #######################
            # Here's Exactly what you should expect to recieve:
            '''
            {"profileName": "Zaid", "controller1": "Xbox One Game Controller (STANDARD GAMEPAD)", "controller2": "null", "associated_mappings": {"0": {"buttons": {"0": "None", "1": "None", "2": "None", "3": "None", "4": "heave_up", "5": "open_claw", "6": "heave_down", "7": "close_claw", "8": "None", "9": "None", "10": "None", "11": "None", "12": "None", "13": "None", "14": "None", "15": "None", "16": "None"}, "axes": {"0": "sway", "1": "surge", "2": "yaw", "3": "pitch"}, "deadzones": {"0": "0", "1": "0", "2": "0", "3": "0"}}, "1": {}}}
            '''
            #######################

            mappings  = json.loads(request.data)
            Pname = mappings["profileName"]
            
            #######################
            # This code below is not necessary
            #######################
            newProfile = {Pname: 
                          {
                                "controller1" : mappings["controller1"],
                                0: {}                                   
                          }}
            
            newProfile[Pname][0]["axes"] = addAxes(0)
            

            #######################
            # This code below is not necessary
            #######################
            if mappings["controller2"] == "null": # controller 2 is not in use
                newProfile[Pname]["controller2"] = None
                newProfile[Pname][1] = {"buttons": {"actions": {}}, "axes": {"actions": {}, "deadzones": {}}}
            else:
                newProfile[Pname]["controller2"] = mappings["controller2"]
                newProfile[Pname][1] = addAxes(1)

            #######################
            # This code below is not necessary
            #######################
            def addAxes(number):
                buttons = mappings["associated_mappings"][str(number)]["buttons"]
                axes = mappings["associated_mappings"][str(number)]["axes"]
                deadzones = mappings["associated_mappings"][str(number)]["deadzones"]
                
                controllermappings = {"buttons": {buttons},"axes": {"actions": {}, "deadzones": {}}}

                for i, key in enumerate(axes):
                    controllermappings["axes"]["actions"][str(i)] = axes[i]
                    controllermappings["axes"]["deadzones"][str(i)] = deadzones[i]   

                return controllermappings  

            #######################
            # I think the below line is better replaced by this:
            # current_directory = os.path.dirname(os.path.abspath(__file__))
            # os.makedirs(os.path.dirname(f"{current_directory}/config/profiles.json"), exist_ok=True)
            #######################
            os.makedirs("config", exist_ok=True)

            #######################
            # I think it may be better store all profiles in a list
            # Everything you recieve a mapping from the database, you can add it to the list
            # In the below function, you return the file. This is not necessary or recommended
            # I suggest that you load the list from the file in the delProfile function 
            # then you can return a new list that doesn't include the profile you want to delete
            # Also, pass in a filepath argument of f"{current_directory}/config/profiles.json"
            #######################
            result, f = delProfile(Pname) # deletes previous profile with that name
            
            #######################
            # Here, you can call json.dumps() on the profiles returned by delProfile
            #######################
            JSONprofile = json.dumps(newProfile)

            #######################
            # Open the file here using:
            # with open(f"{current_directory}/config/profiles.json", "w") as f:
            #     f.write(json.dumps(<whatever you call the json dump of the profiles>)) # writes new profile in JSON format
            #######################
            f.write(JSONprofile) # writes new profile in JSON format
            response.result = JSONprofile
            
            return response                


        elif request.state == 1: #We are looking to load mapping into GUI from database

            mappings = []

            #######################
            # Define a boolean for profile exists
            #######################
            
            #######################
            # use:
            # current_directory = os.path.dirname(os.path.abspath(__file__))
            # os.makedirs(os.path.dirname(f"{current_directory}/config/profiles.json"), exist_ok=True)
            #######################

            try:
                #######################
                # use f"{current_directory}/config/profiles.json"
                #######################
                with open("/app/src/beaumont_pkg/beaumont_pkg/config/profiles.json", "r") as f:
                    profiles  = json.loads(f.read())
                    
                    #######################
                    # Code below is not necessary
                    #######################
                    # only the buttons, axes and deadzones are need to be returned
                    rmap = {0: {"buttons": {}, "axes": {}, "deadzones": {}}, 1: {"buttons": {}, "axes": {}, "deadzones": {}}}
                    
                    if request.data in profiles:
                        secelectedP = profiles[request.data] 
                        for controller in [0,1]:
                            rmap[controller]["axes"] = secelectedP[controller]["axes"]["actions"] #the information is received in a different format than how it was stored
                            rmap[controller]["deadzones"] = secelectedP[controller]["axes"]["deadzones"]                            
                        response.result = json.dumps(rmap) # transforms dict into JSON
                    else:
                        raise FileNotFoundError

                    #######################
                    # for each item in profiles (which is a list of mappings), determine if mapping["profileName"] == request.data
                    # If it is, set the profile_exists boolean to True and do response.result = json.dumps(mapping["associated_mappings"])
                    #######################
            
            except FileNotFoundError:
                #######################
                # replace the two lines below with `pass`
                # The two lines below can be taken outside the try except block and executed if the profile_exists boolean is False
                #######################
                defaultmap = {"axes": {}, "buttons": {}, "deadzones": {}}
                response.result = json.dumps({0: defaultmap, 1: defaultmap}) #Turn the JSON object into a string

            #######################
            # use f"{current_directory}/config/profiles.json"
            #######################
            
            return response


    def profiles_list_callback(self, request, response):

        #######################
        # use:
        # current_directory = os.path.dirname(os.path.abspath(__file__))
        # os.makedirs(os.path.dirname(f"{current_directory}/config/profiles.json"), exist_ok=True)
        #######################

        if request.state == 0: # The requested profile should be deleted

            #######################
            # delProfile should not return the profiles list (with the necessery profile removed) and the string "Profile Not Found" or "Profile Deleted"
            # You can add the following two lines under:
            # with open(f"{current_directory}/config/profiles.json", "w") as f:
            #     f.write(json.dumps(profiles)) 
            #######################
            response.result, file = Delprofile(request.data)
            return response

        elif request.state == 1: #We only want to read the profiles

            #######################
            # write:
            # profilesNames = []
            #######################

            try:
                #######################
                # use f"{current_directory}/config/profiles.json"
                #######################
                with open("/app/src/beaumont_pkg/beaumont_pkg/config/profiles.json", "r") as f:
                    #######################
                    # use
                    #profiles  = json.loads(f.read())          
                    #######################
                    response.result = f.read() # file is already in JSON format

                    #######################
                    # you can do the following:
                    # for index, mapping in enumerate(profiles):
                    #     profilesNames.append({"id": index, 
                    #                         "name":mapping["profileName"],
                    #                         "controller1": mapping["controller1"],
                    #                         "controller2": mapping["controller2"]})
                    #######################

            except FileNotFoundError:
                #######################
                # replace the line below with `pass`
                #######################
                response.result = json.dumps("") 
            
            #######################
            # Use response.result = json.dumps(profilesNames)
            #######################

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
