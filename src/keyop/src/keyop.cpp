#include "ros/ros.h"
#include "keyop/Keyop.h"

#include "SDL/SDL.h"
#include <string>
#include <map>
#include <iostream>

std::string forward;
std::string right;
std::string back;
std::string left;

std::string linear_accelerate;
std::string linear_deaccelerate;
std::string angular_accelerate;
std::string angular_deaccelerate;

void initParams(ros::NodeHandle n) {
    n.getParam("/forward", forward);
    n.getParam("/right", right);
    n.getParam("/back", back);
    n.getParam("/left", left);
    n.getParam("/linear_accelerate", linear_accelerate);
    n.getParam("/linear_deaccelerate", linear_deaccelerate);
    n.getParam("/angular_accelerate", angular_accelerate);
    n.getParam("/angular_deaccelerate", angular_deaccelerate);
}

/* main */
int main( int argc, char *argv[] ){
	
	std::map<std::string, bool> pressed;
    pressed[forward] = false;
    pressed[right] = false;
    pressed[back] = false;
    pressed[left] = false;
    pressed[linear_accelerate] = false;
    pressed[linear_deaccelerate] = false;
    pressed[angular_accelerate] = false;
    pressed[angular_deaccelerate] = false;
	        
    SDL_Event event;
    bool quit = false;
    
    /* Initialise SDL */
    if( SDL_Init( SDL_INIT_VIDEO ) < 0){
        fprintf( stderr, "Could not initialise SDL: %s\n", SDL_GetError() );
        exit( -1 );
    }

    /* Set a video mode */
    if( !SDL_SetVideoMode( 320, 200, 0, 0 ) ){
        fprintf( stderr, "Could not set video mode: %s\n", SDL_GetError() );
        SDL_Quit();
        exit( -1 );
    }

    /* Set name of window */
    SDL_WM_SetCaption("Keyboard controller", 0);

    /* Enable Unicode translation */
    SDL_EnableUNICODE( 1 );



	/* ROS */
    ros::init(argc, argv, "keyop");
	
	ros::NodeHandle n;

    initParams(n);
	
    ros::Publisher keyop_pub = n.advertise<keyop::Keyop>("/keyop/Keyop", 1000);
	
    keyop::Keyop msg;


    /* Loop until an SDL_QUIT event is found */
    while( !quit ){

        /* Poll for events */
        while( SDL_PollEvent( &event ) ){
            bool keydown = false;
            switch( event.type ){
                /* Keyboard event */
                /* SDL_QUIT event (window close) */
                case SDL_QUIT:
                    ROS_INFO("hej");
                    quit = true;
                    break;
                case SDL_KEYDOWN:
                	keydown = true;
                case SDL_KEYUP:
                	if (pressed.count(SDL_GetKeyName(event.key.keysym.sym))) {
                		pressed[SDL_GetKeyName(event.key.keysym.sym)] = keydown;
                	}
                    break;
                default:
                    break;
            }

            if (quit) {
                break;
            }
            
            msg.forward = pressed[forward];
            msg.right = pressed[right];
            msg.back = pressed[back];
            msg.left = pressed[left];
            msg.linear_accelerate = pressed[linear_accelerate];
            msg.linear_deaccelerate = pressed[linear_deaccelerate];
            msg.angular_accelerate = pressed[angular_accelerate];
            msg.angular_deaccelerate = pressed[angular_deaccelerate];
            
            keyop_pub.publish(msg);
            
            ros::spinOnce();
        }

    }

    /* Clean up */
    SDL_Quit();
    exit( 0 );
}
