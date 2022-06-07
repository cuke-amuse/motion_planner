////
////  main.cpp
////  TestOpenGL
////
////  Created by HJKBD on 8/20/16.
////  Copyright © 2016 HJKBD. All rights reserved.
////
//
#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>

#include <picojson.h>
#include <layout.h>
#include <display.h>
#include <globals.h>
#include <model.h>
#include <search.h>
#include <inference.h>

#include <time.h>
#include <unistd.h>
#include <iomanip>
#include "KdTree.hpp"
#include <decisionmaking2.h>
//#include <decisionmaking.h>
#include <util.h>

using namespace std;

GLFWwindow *window = NULL;

Display display = Display();
ofstream myfile;

int main(int argc, char* argv[]) 
{
    //load the map for running testing examples
    myfile.open ("intention.txt");
    std::string worldname = "road";
    // init cyber framework
       char opt;
        while((opt = getopt(argc, argv, "he:t:n:o:")) != -1) {
        switch(opt) {
            case 'h':
                printf("Usage: ./FileEnDeCryptor -i <InputFile> -t <E/D> -n <Num> -o <OutFile>\n");
                return 0;
            case 'e':
                worldname = string(optarg);
                break;
        }
    }

    Layout layout = Layout(worldname);
    Model model(layout);
    display.setColors(model.getCars());
    
    int SCREEN_WIDTH = layout.getWidth();
    int SCREEN_HEIGHT = layout.getHeight();
    
    Car* mycar = model.getHost();
    vector<Car*> cars = model.getCars();
//    std::cout<<car->isHost()<<std::endl;
    string title = Globals::constant.TITLE;
    begin_graphics(SCREEN_WIDTH, SCREEN_HEIGHT, title);
    std::cout<<typeid(*mycar).name()<<std::endl;
    //loop util the user closes the window
    //bool gameover = false;
    
    bool over = false;
    DecisionAgent2 decision;
    vector<vec2f> mypath;
    
    vector<int> carintentions;
    for (int i = 0; i < model.getOtherCars().size(); i++) {
        carintentions.push_back(1);
    }

    bool success = decision.getPath(model, mypath, carintentions);
    
    vector<vector<vec2f>> mypaths = decision.getPaths();
    std::pair<std::string, vec2f> actionset;
    string filename = "coop";

// the change for car is mandatory
    bool change = true;
    srand(time(NULL));

    while(!glfwWindowShouldClose(window)) {
        glClearColor(1.0f, 1.0f, 1.0f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        Display::drawGoal(model.getFinish());
        Display::drawBlocks(model.getBlocks());
        Display::drawLine(model.getLine());
        
        for(auto p : mypaths)
            drawPolygon(p);
        
        display.drawCar(model.getHost());
        display.drawOtherCar(model.getOtherCars());
        
        if (!gameover(model)) {
            //update cars here for further processing
           for (Car* car : cars) {
               //my car moves
               if (car == mycar) {
                   //destination reaches, generate new paths
                   if (mypath.size() == 0 || abs(mycar->getPos().x- mypath[mypath.size()-1].x) < 10) {
                       // achieve to the goal
                       success = decision.getPath(model, mypath, carintentions);
                       change = decision.isChangeRequired(car, model);
                       mypaths = decision.getPaths();
                       if (!success && change) {
                           carintentions = infer(model);
                           mypath.clear();
                           decision.ApplyAction(model, 0, "dec");
                       } else {
                           car->autonomousAction(mypath, model, NULL);
                           car->update();
                       }
                   } else {
                       car->autonomousAction(mypath, model, NULL);
                       car->update();
                   }
                   drawPolygon(mypath);
               }  else  {    // other car move
                   car->autonomousAction(mypath, model, NULL);
                   car->update();
               }
            }
       }

        glfwSwapBuffers(window);
        glfwPollEvents();
        over = (gameover(model)||glfwWindowShouldClose(window));
        Display::sleep(0.05);
    }
    if (model.checkVictory())
        std::cout<<"The car win"<<endl;
    else
        std::cout<<"You lose the car game" <<endl;
    glfwTerminate();
    myfile.close();
    return 1;
}

