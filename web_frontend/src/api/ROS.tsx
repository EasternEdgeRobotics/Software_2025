import { useAtom } from "jotai";
import ROSLIB, { Ros } from "roslib";
import {
    IsROSConnected, ROSIP, CameraURLs, ThrusterMultipliers, RequestingConfig,
    RequestingProfilesList, Mappings, ProfilesList, CurrentProfile, ControllerInput,
    RequestingCameraURLs, DiagnosticsData1, DiagnosticsData2
} from "./Atoms";
import React from "react";

let initial_page_load = true
let first_input_sent = false

export function InitROS() {
    const [RosIP] = useAtom(ROSIP); // The ip of the device running the rosbridge server (will be the Pi4 in enclosure)
    const [, setIsRosConnected] = useAtom(IsROSConnected); // Used in BotTab, indicates if we are communicating with the rosbridge server
    const [thrusterMultipliers, setThrusterMultipliers] = useAtom(ThrusterMultipliers);
    const [requestingConfig, setRequestingConfig] = useAtom(RequestingConfig); // Used for requesting controller mappings data from the database running in the Pi4
    const [requestingProfilesList, setRequestingProfilesList] = useAtom(RequestingProfilesList); // Used for requesting profiles data from the database running in the Pi4
    const [requestingCameraURLs, setRequestingCameraURLs] = useAtom(RequestingCameraURLs); // Used for requsting camera URL data from databse running in the PI4
    const [mappings, setMappings] = useAtom(Mappings); // Controller mappings
    const [, setProfilesList] = useAtom(ProfilesList); // The known list of pilot profiles
    const [cameraURLs, setCameraURLs] = useAtom(CameraURLs);
    const [currentProfile,] = useAtom(CurrentProfile);
    const [controllerInput, setControllerInput] = useAtom(ControllerInput); // The current controller input from the pilot
    const [, setDiagnosticsData1] = useAtom(DiagnosticsData1);
    const [, setDiagnosticsData2] = useAtom(DiagnosticsData2);

    const [hasRecieved, setHasRecieved] = React.useState<boolean>(false);
    const [ros, setRos] = React.useState<Ros>(new ROSLIB.Ros({}));

    ros.on("connection", () => {
        console.log("ROS Connected!");
        setIsRosConnected(true);
        // thrusterRequestService.callService(0, (response: {power: number, surge: number, sway: number, heave: number, pitch: number, yaw: number}) =>
        //     {setThrusterMultipliers([response.power, response.surge, response.sway, response.heave, response.pitch, response.yaw]);});
        setHasRecieved(true);
    });
    ros.on("close", () => {
        console.log("ROS Disconnected!");
        setIsRosConnected(false);
        setHasRecieved(false);
    });
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    ros.on("error", () => { }); // to prevent page breaking

    React.useEffect(() => {
        ros.connect(`ws://${RosIP}:9090`);
        setInterval(() => {
            if (!ros.isConnected) {
                setRos(new ROSLIB.Ros({}));
                ros.connect(`ws://${RosIP}:9090`);
            }
        }, 1000);
    }, []);

    // Create a publisher on the "/thruster_multipliers" ros2 topic using a custom EER message interface 
    const thrusterValsTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/thruster_multipliers",
        messageType: "eer_interfaces/ThrusterMultipliers"
    });

    // Publish the new power multipliers whenever they change
    React.useEffect(() => {
        const thrusterVals = new ROSLIB.Message({
            power: thrusterMultipliers[0],
            surge: thrusterMultipliers[1],
            sway: thrusterMultipliers[2],
            heave: thrusterMultipliers[3],
            pitch: thrusterMultipliers[4],
            yaw: thrusterMultipliers[5]
        });
        if (hasRecieved) thrusterValsTopic.publish(thrusterVals);
    }
        , [thrusterMultipliers]);

    // Create a publisher on the "/pilot_input" ros2 topic, using the default String message which will be used from transporting JSON data
    const controllerInputTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/pilot_input",
        messageType: "eer_interfaces/PilotInput",
        queue_size: 1
    });

    // Publish the new controller input whenever it changes (10 Hz)
    React.useEffect(() => {
        const controllerInputVals = new ROSLIB.Message({
            surge: controllerInput[0],
            sway: controllerInput[1],
            heave: controllerInput[2],
            pitch: controllerInput[3],
            roll: controllerInput[4],
            yaw: controllerInput[5],
            open_claw: controllerInput[6] ? true : false,
            close_claw: controllerInput[7] ? true : false,
            heave_up: controllerInput[8] ? true : false,
            heave_down: controllerInput[9] ? true : false,
            pitch_up: controllerInput[10] ? true : false,
            pitch_down: controllerInput[11] ? true : false,
            roll_cw: controllerInput[12] ? true : false,
            roll_ccw: controllerInput[13] ? true : false,
            turn_front_servo_cw: controllerInput[14] ? true : false,
            turn_front_servo_ccw: controllerInput[15] ? true : false,
            turn_back_servo_cw: controllerInput[16] ? true : false,
            turn_back_servo_ccw: controllerInput[17] ? true : false,
            brighten_led: controllerInput[18] ? true : false,
            dim_led: controllerInput[19] ? true : false,
            turn_stepper_cw: controllerInput[20] ? true : false,
            turn_stepper_ccw: controllerInput[21] ? true : false,
            read_outside_temperature_probe: controllerInput[22] ? true : false,
            enter_auto_mode: controllerInput[23] ? true : false
        });
        controllerInputTopic.publish(controllerInputVals);
        first_input_sent = true;
    }
        , [controllerInput]);

    // Create a ROS service on the "/profile_config" topic using a custom EER service interface
    const configClient = new ROSLIB.Service({
        ros: ros,
        name: "/profile_config",
        serviceType: "eer_interfaces/Config"
    });

    // Run the block of code below whenever the RequestingConfig global state is changed
    React.useEffect(() => {
        if (requestingConfig.state == 0) { // State 0 indicates that we are saving mappings for a certain profile to the database
            const request = new ROSLIB.ServiceRequest({
                state: requestingConfig.state,
                data: JSON.stringify({
                    "profileName": requestingConfig.profileName, "controller1": requestingConfig.controller1,
                    "controller2": requestingConfig.controller2, "associated_mappings": mappings
                })
            }); // Turn the JSON object into a string to send over ROS
            // eslint-disable-next-line @typescript-eslint/no-empty-function
            configClient.callService(request, () => { });
        }
        else if (requestingConfig.state == 1) { // State 1 indicates that we are requesting mappings for a certain profile from the database
            const request = new ROSLIB.ServiceRequest({
                state: requestingConfig.state,
                data: requestingConfig.profileName
            });

            // When a response is recieved for a ROS service request, it is expected to be run through a "callback function". In this case, the function definition is being
            // used as a parameter to configClient.callService instead of a reference to the function.
            configClient.callService(request, function (result) {
                const databaseStoredMappings = JSON.parse(result.result); // Turn the recieved string into a JSON object
                const tmp = mappings;

                // For each controller, store the mappings in a temporary object

                for (let i = 0; i < 2; i++) {
                    if ([requestingConfig.controller1, requestingConfig.controller2][i] == "recognized") {
                        tmp[i] = { "buttons": {}, "axes": {}, "deadzones": {} };
                        tmp[i]["buttons"] = databaseStoredMappings[i]["buttons"];
                        tmp[i]["axes"] = databaseStoredMappings[i]["axes"];
                        tmp[i]["deadzones"] = databaseStoredMappings[i]["deadzones"];
                    }
                }

                // Set the global Mappings state to the tmp object which now holds the mappings recieved from the database. Note that just running setMappings(databaseStoredMappings) didn't work
                setMappings(tmp);
            });
        }
        if (requestingConfig.state != 2) { // State 2 doesn't do anything, and is used as the default state. Whenever a service call is done for state 0 or 1, RequestingConfig returns to state 2
            setRequestingConfig({ state: 2, profileName: "default", controller1: "null", controller2: "null" });
        }
    }
        , [requestingConfig]);


    // Create a ROS service on the "/profile_list" topic using a custom EER service interface
    const profilesListClient = new ROSLIB.Service({
        ros: ros,
        name: "/profiles_list",
        serviceType: "eer_interfaces/Config"
    });

    // Run the block of code below whenever the RequestingProfilesList global state is changed
    React.useEffect(() => {
        if (requestingProfilesList == 0) { // State 0 indicates that we would like to delete a profile from the database.
            // Note that profiles are created when RequestingConfig state 0 is called and the database doesn't recognize the name of the profile coming from the GUI
            const request = new ROSLIB.ServiceRequest({
                state: 0,
                data: currentProfile
            });
            profilesListClient.callService(request, function (result) {
                console.log(result.result); // Should return "Profile Deleted" or "Profile Not Found"
            });
        }
        else if (requestingProfilesList == 1) { // State 1 indicates that we are requesting the entire list of profiles from the database
            const request = new ROSLIB.ServiceRequest({
                state: 1,
                data: JSON.stringify(mappings)
            }); // Turn the mappings JSON object into a string to send over ROS

            profilesListClient.callService(request, function (result) {
                if (result.result != "[]") { // Do not load the result if there are no profiles stored (i.e. empty list is returned)
                    setProfilesList(JSON.parse(result.result));
                }
                else { // If we recieve an empty list, then just set the profile to "default". The pilot will have to create a profile on the fly that will only be saved locally, and will be gone on page reload
                    setProfilesList([{ id: 0, name: "default", controller1: "null", controller2: "null" }]);
                }
            });
        }
        setRequestingProfilesList(2);
    }
        , [requestingProfilesList]);

    // Create a ROS service on the "/camera_urls" topic using a custom EER service interface
    const cameraURLsClient = new ROSLIB.Service({
        ros: ros,
        name: "/camera_urls",
        serviceType: "eer_interfaces/Cameras"
    });

    // Run the block of code below whenever the RequestingCameraURLs global state is changed
    React.useEffect(() => {
        if (requestingCameraURLs == 0) { // State 0 indicates that we would like to save camera URLs into database
            const request = new ROSLIB.ServiceRequest({
                state: 0,
                camera_urls_json: JSON.stringify(cameraURLs),
            });
            cameraURLsClient.callService(request, function (result) { null; });
        }
        else if (requestingCameraURLs == 1) { // State 1 indicates that we are looking to fetch camera URLs from database
            const request = new ROSLIB.ServiceRequest({
                state: 1,
            }); // What's in the data field is not important in this case
            cameraURLsClient.callService(request, function (result) {
                if (result.camera_urls_json != "[]") {
                    setCameraURLs(JSON.parse(result.camera_urls_json));
                }
                else {
                    console.log("No camera URLs stored in database")
                }
            });
        }
        setRequestingCameraURLs(2);
    }
        , [requestingCameraURLs]);

    if (initial_page_load && first_input_sent) {

        const DiagnosticsData1Listener = new ROSLIB.Topic({
            ros: ros,
            name: "/diagnostics_data_1",
            messageType: "std_msgs/String"
        })

        DiagnosticsData1Listener.subscribe(function (message) {

            setDiagnosticsData1((message as any).data);
        })

        const DiagnosticsData2Listener = new ROSLIB.Topic({
            ros: ros,
            name: "/diagnostics_data_2",
            messageType: "std_msgs/String"
        })

        DiagnosticsData2Listener.subscribe(function (message) {
            setDiagnosticsData2((message as any).data);
        })

        initial_page_load = false
    }



    return (null);
}