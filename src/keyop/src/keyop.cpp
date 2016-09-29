#include "ros/ros.h"
#include "keyop/Keyop.h"

#include "SDL/SDL.h"
#include <string>
#include <map>
#include <iostream>

std::string forward = "up";
std::string right = "right";
std::string back = "down";
std::string left = "left";

std::string accelerate = "q";
std::string deaccelerate = "a";


/* main */
int main( int argc, char *argv[] ){
	
	std::map<std::string, bool> pressed;
    pressed[forward] = false;
    pressed[right] = false;
    pressed[back] = false;
    pressed[left] = false;
    pressed[accelerate] = false;
    pressed[deaccelerate] = false;
	        
    SDL_Event event;
    int quit = 0;
    
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
	
    ros::Publisher keyop_pub = n.advertise<keyop::Keyop>("/keyop/Keyop", 1000);
	
    keyop::Keyop msg;


    /* Loop until an SDL_QUIT event is found */
    while( !quit ){

        /* Poll for events */
        while( SDL_PollEvent( &event ) ){
            bool keydown = false;
            switch( event.type ){
                /* Keyboard event */
                /* Pass the event data onto PrintKeyInfo() */
                case SDL_KEYDOWN:
                	keydown = true;
                case SDL_KEYUP:
                	if (pressed.count(SDL_GetKeyName(event.key.keysym.sym))) {
                		pressed[SDL_GetKeyName(event.key.keysym.sym)] = keydown;
                	}
                    break;

                /* SDL_QUIT event (window close) */
                case SDL_QUIT:
                    quit = 1;
                    break;

                default:
                    break;
            }
            
            msg.forward = pressed[forward];
            msg.right = pressed[right];
            msg.back = pressed[back];
            msg.left = pressed[left];
            msg.accelerate = pressed[accelerate];
            msg.deaccelerate = pressed[deaccelerate];
            
            keyop_pub.publish(msg);
            
            ros::spinOnce();
        }

    }

    /* Clean up */
    SDL_Quit();
    exit( 0 );
}
