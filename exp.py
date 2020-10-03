from params.central_params import create_joystick_params        

def test_joystick():
    flag =0
    if flag == 0:
        from joystick.joystick_RVO import JoystickRVO as Joystick
    else:
        from joystick.joystick_social_force import JoystickSocialForce as Joystick
        
    joystick_params = create_joystick_params()
    """start the joystick process"""
    J = Joystick()
    J.init_send_conn()
    J.init_recv_conn()
    # first listen() for the episode names
    assert(J.get_all_episode_names())
    episodes = J.get_episodes()
    # we want to run on at least one episode
    assert(len(episodes) > 0)
    #ep_title = episodes[0]
    for ep_title in episodes:
        print("Waiting for episode: {}".format(ep_title))
        # second listen() for the specific episode details
        J.get_episode_metadata()
        assert(J.current_ep and J.current_ep.get_name() == ep_title)
        J.init_control_pipeline()
        J.update_loop()

if __name__ == '__main__':
    print("Joystick")
    test_joystick()
