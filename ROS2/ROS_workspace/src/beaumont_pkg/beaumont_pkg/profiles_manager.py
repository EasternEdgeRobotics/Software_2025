from eer_messages.srv import Config
import os

import rclpy
from rclpy.node import Node
import json

# class Base(DeclarativeBase):
#     pass

# #Define the Profiles database schema
# class Profile(Base):
#     __tablename__ = "profiles"
#     id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
#     name: Mapped[str] = mapped_column(unique=True)
#     controller1: Mapped[str] = mapped_column()
#     controller2: Mapped[str] = mapped_column(nullable=True)
    
#     def dict(self):
#         return {"id": self.id, "name": self.name, "controller1": self.controller1, "controller2": self.controller2}

# #Define the Mappings database schema
# class Mapping(Base):
#     __tablename__ = "mappings"
#     id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
#     name: Mapped[str] = mapped_column(ForeignKey("profiles.name", ondelete="cascade"))
#     controller: Mapped[str] = mapped_column()
#     controllerNumber: Mapped[int] = mapped_column()
#     button: Mapped[int] = mapped_column()
#     action: Mapped[str] = mapped_column()
#     isAxis: Mapped[bool] = mapped_column()
#     deadzone: Mapped[float] = mapped_column(nullable=True)
#     def dict(self):
#         return {"id": self.id, "name": self.name, "controller": self.controller,"controllerNumber": self.controllerNumber , "button": self.button, "action": self.action , "isAxis": self.isAxis, "deadzone": self.deadzone}

# class PowerPreset(Base):
#     __tablename__ = "powerPresets"
#     name: Mapped[str] = mapped_column(primary_key=True)
#     power: Mapped[int] = mapped_column()
#     surge: Mapped[int] = mapped_column()
#     sway: Mapped[int] = mapped_column()
#     heave: Mapped[int] = mapped_column()
#     pitch: Mapped[int] = mapped_column()
#     roll: Mapped[int] = mapped_column()
#     yaw: Mapped[int] = mapped_column()

# class Camera(Base):
#     __tablename__ = "cameras"
#     id: Mapped[int] = mapped_column(primary_key=True)
#     url1: Mapped[str] = mapped_column()
#     url2: Mapped[str] = mapped_column()
#     url3: Mapped[str] = mapped_column()
#     url4: Mapped[str] = mapped_column()
#     def dict(self):
#         return {"url1":self.url1, "url2":self.url2, "url3":self.url3, "url4":self.url4}

# engine = create_engine("sqlite:///config.db")

# def mappings_list_to_mappings_json(mappings_list):
#"""Takes in a list of mappings for a certain profile from the database and turns in into a JSON"""

#     controller_1_json_mappings = {"axes": {}, "buttons": {}, "deadzones": {}}
#     controller_2_json_mappings = {"axes": {}, "buttons": {}, "deadzones": {}}

#     def obtain_bindings(binding, _json_mappings):
#         if binding["isAxis"] == True:
#             _json_mappings["axes"][binding["button"]] = binding["action"]
#             if binding["deadzone"]==None:
#                 binding["deadzone"] = 0
#             _json_mappings["deadzones"][binding["button"]] = str(binding["deadzone"])
#         else:
#             _json_mappings["buttons"][binding["button"]] = binding["action"]
#         return _json_mappings
            
#     for mapping in mappings_list:

#         if mapping["controllerNumber"] == 0:
#             controller_1_json_mappings = obtain_bindings(mapping, controller_1_json_mappings)
#         elif mapping["controllerNumber"] == 1:
#             controller_2_json_mappings = obtain_bindings(mapping, controller_2_json_mappings)

#     return {0: controller_1_json_mappings, 1: controller_2_json_mappings}

# def mappings_json_to_mappings_list(profile_name, controller_name, mappings_json, controller_number):
#     """Takes in a JSON dictionary for a certain profile and turns in into a mappings list to store in database"""

#     buttons_dict = mappings_json["buttons"]
#     for i, key in enumerate(list(buttons_dict.keys())):
#         new_mapping = Mapping(name = profile_name, controller = controller_name, controllerNumber = controller_number, button = i, action = buttons_dict[str(i)], isAxis = False) 
#         session.add(new_mapping)

#         session.commit()
    
#     axes_dict = mappings_json["axes"]
#     deadzones_dict = mappings_json["deadzones"]
#     for i, key in enumerate(list(axes_dict.keys())):
#         if (float(deadzones_dict[str(i)]) == 0): 
#             new_mapping = Mapping(name = profile_name, controller = controller_name, controllerNumber = controller_number, button = i, action = axes_dict[str(i)], isAxis = True)
#         else:
#             new_mapping = Mapping(name = profile_name, controller = controller_name, controllerNumber = controller_number, button = i, action = axes_dict[str(i)], isAxis = True, deadzone = float(deadzones_dict[str(i)]))
            
#         session.add(new_mapping)

#         session.commit()

def delProfile(profileDel):
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

            mappings  = json.loads(request.data)
            Pname = mappings["profileName"]
            

            newProfile = {Pname: 
                          {
                                "controller1" : mappings["controller1"],
                                0: {}                                   
                          }}
            
            newProfile[Pname][0]["axes"] = addAxes(0)
            

            if mappings["controller2"] == "null": # controller 2 is not in use
                newProfile[Pname]["controller2"] = None
                newProfile[Pname][1] = {"buttons": {"actions": {}}, "axes": {"actions": {}, "deadzones": {}}}
            else:
                newProfile[Pname]["controller2"] = mappings["controller2"]
                newProfile[Pname][1] = addAxes(1)


            def addAxes(number):
                buttons = mappings["associated_mappings"][str(number)]["buttons"]
                axes = mappings["associated_mappings"][str(number)]["axes"]
                deadzones = mappings["associated_mappings"][str(number)]["deadzones"]
                
                controllermappings = {"buttons": {buttons},"axes": {"actions": {}, "deadzones": {}}}

                for i, key in enumerate(axes):
                    controllermappings["axes"]["actions"][str(i)] = axes[i]
                    controllermappings["axes"]["deadzones"][str(i)] = deadzones[i]   

                return controllermappings  


            os.makedirs("config", exist_ok=True)
            result, f = delProfile(Pname) # deletes previous profile with that name
            
            JSONprofile = json.dumps(newProfile)
            f.write(JSONprofile) # writes new profile in JSON format
            response.result = JSONprofile
            
            return response                

            
            # message = json.loads(request.data) #Turn the recieved string into a JSON object (i.e. Python dictionary)

            # query = session.query(Profile).filter(Profile.name == message["profileName"]) # Remove the profile if it exists as it will be recreated
            # if query.count() == 1: #i.e. profile exists
            #     query.delete()
            #     session.commit()

            # if (message["controller2"] == "null"):
            #     new_profile = Profile(name=message["profileName"], controller1 = message["controller1"])
            # else:
            #     new_profile = Profile(name=message["profileName"], controller1 = message["controller1"], controller2 = message["controller2"])

            # session.add(new_profile)
            # session.commit()

            # mappings_json_to_mappings_list(message["profileName"], message["controller1"], message["associated_mappings"]["0"], 0) #This function does not return a value, it directly modifies the database

            # if (message["controller2"] != "null"):
            #     mappings_json_to_mappings_list(message["profileName"], message["controller2"], message["associated_mappings"]["1"], 1)
            # response.result = json.dumps(message) 

            #==========================DEBUG===========================
            # for i in range(session.query(Mapping).filter(Mapping.name == message["profileName"]).count()):
            #    self.get_logger().info(f"{session.query(Mapping).filter_by(name = message['profileName']).all()[i].dict()} recieved")
            #==========================================================


        elif request.state == 1: #We are looking to load mapping into GUI from database
            # mappings_list = []
            # for row in session.query(Mapping).all():
            #     if (row.dict()["name"]==request.data): #request.data in this case stores the name of the profile for which mappings are being requested
            #         mappings_list.append(row.dict())
            # mappings_json = mappings_list_to_mappings_json(mappings_list)

            mappings = []

            try:
                with open("/app/src/beaumont_pkg/beaumont_pkg/config/profiles.json", "r") as f:
                    profiles  = json.loads(f.read())
                    
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
            
            except FileNotFoundError:
                defaultmap = {"axes": {}, "buttons": {}, "deadzones": {}}
                response.result = json.dumps({0: defaultmap, 1: defaultmap}) #Turn the JSON object into a string
            
            return response


    def profiles_list_callback(self, request, response):
        if request.state == 0: # The requested profile should be deleted

            response.result, file = Delprofile(request.data)
            return response
            
            # query = session.query(Profile).filter(Profile.name == request.data) #In this case, the request data is expected to only be a string
            # if query.count() == 1: #i.e. profile exists
            #     query.delete()
            #     session.commit()
            #     response.result = "Profile Deleted"
            # else:
            #     response.result = "Profile not found"
        elif request.state == 1: #We only want to read the profiles

            try:
                with open("/app/src/beaumont_pkg/beaumont_pkg/config/profiles.json", "r") as f:
                    response.result = f.read() # file is already in JSON format
            except FileNotFoundError:
                response.result = json.dumps("") 
            
            return response
            # output = []
            # for row in session.query(Profile).all():
            #     output.append(row.dict())
            # response.result = json.dumps(output) #Turn the JSON object into a string
            

    def camera_urls_callback(self, request, response):

        if request.state == 0: # We are looking to load camera URLs into database from GUI

            urls = json.loads(request.data) #Turn the recieved string into a List

            while True:
                if len(urls) < 4:
                    urls.append("http://")
                else:
                    break

            os.makedirs("config", exist_ok=True)
            with open("/app/src/beaumont_pkg/beaumont_pkg/config/camera_urls.json", "w") as f:
                f.write(json.dumps(urls))

            response.result = "Success"

            #==========================DEBUG===========================
            # for i in range(session.query(Camera).count()):
            #     self.get_logger().info(f"{session.query(Camera).all()[i].dict()} recieved")
            #==========================================================

            return response
        
        elif request.state == 1: # We are looking to fetch camera URLs form database into GUI

            outgoing_camera_urls = []

            try:
                with open("/app/src/beaumont_pkg/beaumont_pkg/config/camera_urls.json", "r") as f:
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

