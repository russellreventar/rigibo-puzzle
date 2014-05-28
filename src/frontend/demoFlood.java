package frontend;

import java.io.File;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;


import org.lwjgl.LWJGLException;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.newdawn.slick.opengl.Texture;
import org.newdawn.slick.opengl.TextureLoader;

import physics.collision.shapes.Circle;
import physics.collision.shapes.Polygon;

import physics.dynamics.RigidBody;
import physics.dynamics.RigidBodyInfo;
import physics.dynamics.RigidBodyType;

import physics.dynamics.FixtureDef;
import physics.dynamics.Pool;
import physics.tools.Vec;
import static org.lwjgl.opengl.GL11.*;


public class demoFlood{
	
	//Display Info
	int windowHeight = 800;
	int windowWidth = 720; 
	float wallBorder = 0.0f;
	float wall = 0.0f;
		
	//ball
	float ball_radius = 0.30f;

	float dt = 1/90f;
	float crateSize = 2.3f;
	Vec wallmid_size = new Vec(3.0f,.25f);
	boolean initballMade = false;
	Pool world = new Pool(new Vec(0.0f,-9.8f),true);
	ArrayList<RigidBody> balls = new ArrayList<RigidBody>();
	ArrayList<RigidBody> walls = new ArrayList<RigidBody>();
	int ballCount = 0;
	Texture tex_ball;
	Texture tex_background;
	Texture tex_floor;
    public enum State {
    	RUNNING, PAUSE, RESET
    }
    public State state = State.RUNNING;
    
    public void start(){
    	setupWindow();
    	setupOpenGL();
    	open();
    }
    
    public void setupWindow(){
        
    	try {
            Display.setDisplayMode(new DisplayMode(windowWidth, windowHeight));
            Display.setInitialBackground(200.0f, 200.0f, 200.0f);
            Display.setTitle("Flooding Demo");
            Display.setVSyncEnabled(true);
            Display.setLocation(480,25);
            Display.create();
        } catch (LWJGLException e) {
            e.printStackTrace();
            Display.destroy();
            System.exit(1);
        }  	
    }
    public void renderBall(){

 	   for(RigidBody ball : balls){
 				glPushMatrix();
 				Vec bodyPosition = ball.getPosition().mul(30);
 				glTranslatef(bodyPosition.x, bodyPosition.y, 0);
 				glRotated(Math.toDegrees(ball.getAngle()), 0, 0, 1);
 				tex_ball.bind();
 				glBegin(GL_QUADS);
 				glTexCoord2f(0, 0);glVertex2f(-ball_radius*30, -ball_radius*30); // bottom-left 
 				glTexCoord2f(0, 1);glVertex2f(-ball_radius*30, ball_radius*30); // Upper-left
 				glTexCoord2f(1, 1);glVertex2f(ball_radius*30, ball_radius*30); // Upper-right
 				glTexCoord2f(1, 0);glVertex2f(ball_radius*30, -ball_radius*30); // bottom-right
 				glEnd();
 				glPopMatrix();
 			}
    }
    public void setupOpenGL(){
    	 
    	 
    	 glMatrixMode(GL_PROJECTION);
    	 glOrtho(0, windowWidth/2, 0, windowHeight/2, 1, -1);
         glMatrixMode(GL_MODELVIEW);
         glEnable(GL_TEXTURE_2D);
         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
         glEnable( GL_BLEND );

    }
    public void setupTextures(){
    	tex_background = loadTexture("bg");
    	tex_ball = loadTexture("circle_blue");
    }
    public void open(){
    	setupTextures();
    	setupFloors();
    	doStep();
    	while (!Display.isCloseRequested()) {
        	
        	if(Keyboard.isKeyDown(Keyboard.KEY_ESCAPE))exit();
        	inputListener();
        	
        	if(state == State.RUNNING)doStep();
        	renderDisplay();
            updateDisplay();
        }
    	exit();
    	
    }

    public void updateDisplay(){
    	Display.update();
    	Display.sync(60);
    }
    
    public void closeWindow(){
    	Display.destroy();
    }
    
    public void exit(){
    	Display.destroy();
    	System.exit(0);
    }
    public void doStep() {
    	createBall(ball_radius, 5.0f,14.0f,0.2f,0.1f);
        world.step(dt, 8, 3);
    }
    public void renderDisplay() {
        //if(ballCount > 0){
    	glClear(GL_COLOR_BUFFER_BIT);
    	glPushMatrix();
    	tex_background.bind();
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(0f, 0f); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(0.0f, windowHeight); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(windowWidth, windowHeight); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(windowWidth, 0.0f); // bottom-right
		glEnd();
    	glPopMatrix();
        renderBall();
    
    }

    public void createBall(float radius, float x_pos, float y_pos, float density, float r){
    	
    	 RigidBodyInfo ballDef = new RigidBodyInfo();
         ballDef.position.set(x_pos,y_pos);
         ballDef.type = RigidBodyType.DYNAMIC;
    	
         Circle c = new Circle();
         c.m_radius = radius;
         
         FixtureDef ballFixture = new FixtureDef();
         ballFixture.density = density;
         ballFixture.shape = c;
         ballFixture.restitution = r;
         
         RigidBody ball = world.createBody(ballDef);
         ball.createFixture(ballFixture);
         
         if(initballMade)ballCount++;
         balls.add(ball);
         initballMade = true;
    }
    
    public void createWall(float width, float height, float x_pos, float y_pos, float angle){
    	
   	 	RigidBodyInfo wallDef = new RigidBodyInfo();
        wallDef.position.set(x_pos,y_pos);
        wallDef.type = RigidBodyType.STATIC;
        wallDef.angle = angle;
   	
        Polygon wallShape = new Polygon();
        wallShape.setAsBox(width,height);
        
        FixtureDef wallFixture = new FixtureDef();
        wallFixture.shape = wallShape;
        wallFixture.restitution = 0.3f;
        
        RigidBody wall = world.createBody(wallDef);
        wall.createFixture(wallFixture);
        wall.setFixedRotation(true);
        walls.add(wall);
   }
   
    
    public void inputListener() {

    	 
    	while (Keyboard.next()) {
              
    		
    		//Keyboard
    		if (Keyboard.getEventKeyState()) {
                  

    			switch (Keyboard.getEventKey()) {

                      case Keyboard.KEY_SPACE:
//                    	  createBall(ball_radius, 10.0f,10.0f,0.3f);
                    	  createWall(wallmid_size.x, wallmid_size.y, 10f, 8f, (float)Math.toRadians(45));
                    	//  createWall(wallmid_size.x, wallmid_size.y, 7f, 12f, 0.0f);
                    	  createWall(wallmid_size.x, wallmid_size.y, 4f, 8f, (float)Math.toRadians(-45));
                          break;    
                      case Keyboard.KEY_BACK:
                    	  closeWindow();
                    	  MainMenu sim = new MainMenu(MainMenu.State.D_FLOOD);
                    	  sim.start();
                      case Keyboard.KEY_P:
                    	  if(state == State.RUNNING){
                      		state =State.PAUSE;
                      	}else if(state ==State.PAUSE){
                      		state = State.RUNNING;
                      	}
                    	  break;
                      case Keyboard.KEY_RETURN:
                    	  state = State.RUNNING;
                    	  break;
                      case Keyboard.KEY_R:
                    	  state = State.RESET;
                    	  break;
                    	  
                  }
              }
          }

    		if(initballMade){
    			if (Keyboard.isKeyDown(Keyboard.KEY_S) && !Keyboard.isKeyDown(Keyboard.KEY_D)) {
    				balls.get(ballCount).applyAngularImpulse(+0.01f);
    			} else if (Keyboard.isKeyDown(Keyboard.KEY_D) && !Keyboard.isKeyDown(Keyboard.KEY_S)) {
    				balls.get(ballCount).applyAngularImpulse(-0.01f);
    			}else if (Keyboard.isKeyDown(Keyboard.KEY_A) && Mouse.isButtonDown(0)) {
    				for(RigidBody crate : balls){
    				Vec mousePosition = new Vec(Mouse.getX(),Mouse.getY()).mul(0.5f).mul(1 / 30f);
    				Vec bodyPosition = crate.getPosition();
    				Vec force = mousePosition.sub(bodyPosition);
    				crate.applyForce(force, crate.getPosition());
    				}
    			}
    			//Mouse
    			if (Mouse.isButtonDown(0)) {
    				Vec mousePosition = new Vec(Mouse.getX(),Mouse.getY()).mul(0.5f).mul(1 / 30f);
    				Vec bodyPosition = balls.get(ballCount).getPosition();
    				Vec force = mousePosition.sub(bodyPosition);
    				balls.get(ballCount).applyForce(force, balls.get(ballCount).getPosition());
    			}
    		}
    	}
    
    public void setupFloors(){

          RigidBodyInfo leftWallDef = new RigidBodyInfo();
          leftWallDef.position.set(wall, 0);
          leftWallDef.type = RigidBodyType.STATIC;
          Polygon leftWallShape = new Polygon();
          leftWallShape.setAsBox(0, 1000);
          RigidBody leftWall = world.createBody(leftWallDef);
          FixtureDef leftWallFixture = new FixtureDef();
          leftWallFixture.density = 1;
          leftWallFixture.restitution = 0.3f;
          leftWallFixture.shape = leftWallShape;
          leftWall.createFixture(leftWallFixture);

//          BodyDef rightWallDef = new BodyDef();
//          rightWallDef.position.set(((windowWidth/30)/2)-wall, 0);
//          rightWallDef.type = BodyType.STATIC;
//          PolygonShape rightWallShape = new PolygonShape();
//          rightWallShape.setAsBox(0, 1000);
//          Body rightWall = world.createBody(rightWallDef);
//          FixtureDef rightWallFixture = new FixtureDef();
//          rightWallFixture.density = 1;
//          rightWallFixture.restitution = 0.3f;
//          rightWallFixture.shape = rightWallShape;
//          rightWall.createFixture(rightWallFixture);
          
          RigidBodyInfo groundDef = new RigidBodyInfo();
          groundDef.position.set(0, wall);
          groundDef.type = RigidBodyType.STATIC;
          Polygon groundShape = new Polygon();
          groundShape.setAsBox(1000, 0);
          RigidBody ground = world.createBody(groundDef);
          FixtureDef groundFixture = new FixtureDef();
          groundFixture.density = 1f;
          groundFixture.restitution = 0.3f;
          groundFixture.shape = groundShape;
          ground.createFixture(groundFixture);
    }
    public Texture loadTexture(String key){
    	try {
			return TextureLoader.getTexture("PNG", new FileInputStream(new File("resources/" + key + ".png")));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
    	return null;
    }
}

