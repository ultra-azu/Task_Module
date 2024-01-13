



#  Edge cases are bool function that given the Shared Data will return True
# if a Edge case has been detected or False if everything is okay.



movement_edge_cases = [
    "on_the_surface_of_the_pool_and_waypoint_is_not_reachable",
    "very_close_to_the_wall",
    "not_enough_baterry_to_reach_waypoint",
    ""
]



def movement_edge_case_callback(shared_data):
    #  1) Check if the Submarine is on the surface of the pool
        #  Check the Depth Sensor to calculate if we are on the surface of the pool
        #  If Yes check if the Waypoint is above the surface. This means that the waypoint 
        # Must be deleted and we must continue with the next waypoint or task.
    

    #  2) Check if the Submarine is very close to the wall
        #  Check the Image Data of the Camera. If there is a straight wall very close to the camera
        #  we must stop the movement and wait for the user to move the submarine away from the wall.
        # Probrably realizing a rotation of the submarine.
    


    #  3) Check if the Submarine has enough battery to reach the waypoint
    #  Check the Battery Data of the Submarine. If the battery is less than 10% we must stop the movement


    #  4) Check if the Submarine movement is unstable
    #     If the submarine Controls parameters are not well defined we must stop the movement.


    



