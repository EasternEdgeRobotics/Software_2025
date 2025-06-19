# Software_2025

[Eastern Edge Robotics](https://www.easternedgerobotics.com/) (EER) is a student-led engineering design team based at the Memorial University of Newfoundland and the Fisheries and Marine Institute. The team competes in the annual MATE ROV Competition. Every year at EER, we build a small remotely-operated vehicle (ROV). This repository is EER's 2025 software package.

This code repository contains code for maintaining EER's two current ROVs, Beaumont and Waterwitch.

## Table of Contents

- [Repository Structure](#repository-structure) 
    - [ros_workspace](#ros_workspace) 
    - [web_frontend](#web_frontend) 
- [How To Contribute](#how_to_contribute) 

## Repository Structure

### ros_workspace
This project uses ROS2 Jazzy.

#### ROS2 Packages
| Package Name         | Description                                                                 |
|----------------------|-----------------------------------------------------------------------------|
| **beaumont_backend**  | Backend code for interfacing with the ROV named Beaumont.                   |
| **waterwitch_backend** | Backend code for interfacing with the ROV named Waterwitch.               |
| **common_backend**    | Common backend ROS nodes for all ROVs.                                      |
| **eer_interfaces**    | Custom ROS2 interfaces used by all packages.                                |
| **waterwitch_frotnend**     | A frontend built using C++ with a GUI interface, meant for driving the ROV named Waterwitch.                          |

More details [here](./ros_workspace/)

### web_frontend
This folder can be thought of as another ROS2 package. It is written in ReactJS and communicates with our backends over ROS2 topics by using roslibjs.

More details [here](./web_frontend/)

## How to Contribute

1. Ensure that you're on the team Discord and Google Drive
2. Complete the [Onboarding Task](https://docs.google.com/document/d/13x00C8hjDYVJlFbLWkDBietP3UNgz9KQ2YJGyHPrInM/edit?usp=drive_link) and share your progress 
3. Pick a task from our [task tracker](https://docs.google.com/spreadsheets/d/1OF3RxeuQIAM3jEYy3F_bVlgd5O9Kzjha2wboch3H_Rw/edit?usp=drive_link) or suggest one based on the team's goal


Appendix A Checklists

Pre-deployment Construction Checklist

Area clear and safe

Power supply is powered off

Cables/ tether are undamaged

Tether strain relief is attached

Check to make sure the electronics are prop-

erly plugged in

Grease and tighten penetrators

Grease the enclosures O-ring

Close the enclosure by tightening screws in

an alternating pattern

When the enclosure is closed, add the vent

plug and tighten

Visually inspect the ROV to check for any

damage and to make sure the ROV is safe

for water

Leak test ROV for 15 minutes

Check for water in the camera tube and in

the enclosure

If there are no leaks: the ROV is ready. Pro-

ceed to deployment

Leaks: Disassemble and inspect ROV

Deployment (Power On)

Pilot sets up topside

Pilot ensures team is attentive

Co-Pilot calls Power On

ROV is connected to power supply

Tether in coiled by deck crew to ensure easy

use

Tether managers place ROV in the water and

check for bubbles going to the surface

If bubbles are identified, take the ROV out of

the water and check for leaks

Pilot calls “Performing thruster test”

Pilot ensures all thrusters are functioning

properly

Pilot checks cameras to ensure proper feed

and positioning

If all systems are okay, proceed to Launch

Procedure

Launch Procedure (Deck Crew)

Pilot calls power on, and power is turned on

with disabled controls

Deck crew calls hands-off

Deck crew removes their hands from the

ROV

Piloting begins

Mid-Mission (Deck Crew)

When the the ROV is visible in the moon

pool, the deck crew calls surface

Pilot calls safe when controls are disabled

Hands-on is called by the deck crew when

they have made contact with the ROV

Loss of Communication

Steps are attempted in order. If a step is

successful, the mission resumes

After 2.5 seconds without topside commu-

nication, thrusters turn off

Co-Pilot inspects tether and surface connec-

tions to the laptop

Pilot attempts rebooting the control system

Co-Pilot attempts rebooting Waterwitch by

cycling power supply

Power down ROV, call Power Off

Manual retrieval of ROV using tether

Confirm no leaks and begin troubleshooting

Isolate the cause and document

Tether Protocol

Tether Manager conducts inspection of

tether

Tether is deployed carefully and monitored

throughout the mission to make adjust-

ments as needed

Tether Manager retrieves the tether at the

end of the mission and coils it into a figure-

eight pattern for storage
