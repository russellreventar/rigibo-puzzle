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

import physics.collision.shapes.Polygon;
import physics.dynamics.RigidBody;
import physics.dynamics.RigidBodyInfo;
import physics.dynamics.RigidBodyType;
import physics.dynamics.FixtureDef;
import physics.dynamics.Pool;
import physics.tools.Vec;
import static org.lwjgl.opengl.GL11.*;

public class demoWheel{


	int windowHeight = 800;
	int windowWidth = 720;
	float wallBorder = 2.4f;
	float wall = 0.0f;
	float dt = 1/60f;
	float crateSize = 1.5f;
	Vec platformSize = new Vec(7.0f,.5f);
	boolean boxcreated = false;
	Pool world = new Pool(new Vec(0.0f,-9.8f),true);
	ArrayList<RigidBody> crates = new ArrayList<RigidBody>();
	RigidBody platform;
	int crateCount = 0;
	Texture crate_tex;
	Texture background;
	Texture tex_loo;
	RigidBody pivot;
	Vec pivot_pos = new Vec(6.5f,7.0f);
	Vec pivot_size = new Vec(4f,0.25f);
    public enum State {
    }
    public State state;

    public void start(){
    	setupWindow();
    	initializeGL();
    	open();
    }
    
    public void setupWindow(){
        
    	try {
            Display.setDisplayMode(new DisplayMode(windowWidth, windowHeight));
            Display.setInitialBackground(200.0f, 200.0f, 200.0f);
            Display.setTitle("Wheel Demo");
            Display.setLocation(480,25);
            Display.setVSyncEnabled(true);
            Display.create();
        } catch (LWJGLException e) {
            e.printStackTrace();
            Display.destroy();
            System.exit(1);
        }  	
    }
    
    public void initializeGL(){
    	 glMatrixMode(GL_PROJECTION);
    	 glOrtho(0, windowWidth/2, 0, windowHeight/2, 1, -1);
         glMatrixMode(GL_MODELVIEW);
         glEnable(GL_TEXTURE_2D);
         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
         glEnable( GL_BLEND );
    }
    
    public void open(){
    	setupFloors();
    	doStep();
    	crate_tex = loadTexture("crate");
    	tex_loo = loadTexture("loo");
    	background = loadTexture("bg");
    	while (!Display.isCloseRequested()) {
        	
        	if(Keyboard.isKeyDown(Keyboard.KEY_ESCAPE))exit();
    		doStep();
    	   	inputListener();
        	renderDisplay();
            updateDisplay();
        }
    	exit();
    }

    public void updateDisplay(){
    	Display.update();
    	Display.sync(60);
    }
    
    public void closeDisplay(){
    	Display.destroy();
    }
    
    public void exit(){
    	Display.destroy();
    	System.exit(0);
    }
    public void doStep() {
    	pivot.setAngularVelocity(1.5f);
        world.step(dt, 8, 3);
    }
    public void renderDisplay() {
        //if(crateCount > 0){
    	glClear(GL_COLOR_BUFFER_BIT);
    	glPushMatrix();
    	background.bind();
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(0f, 0f); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(0.0f, windowHeight); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(windowWidth, windowHeight); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(windowWidth, 0.0f); // bottom-right
		glEnd();
    	glPopMatrix();
        float size = (crateSize)*30;
    	glPushMatrix();
		Vec bodyPositio = pivot.getPosition().mul(30);
		glTranslatef(bodyPositio.x, bodyPositio.y, 0);
		glRotated(Math.toDegrees(pivot.getAngle()), 0, 0, 1);
		//pivot_tex.bind();
		tex_loo.bind();
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(-(pivot_size.x)*30, -pivot_size.y*30); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(-pivot_size.x*30, pivot_size.y*30); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(pivot_size.x*30, pivot_size.y*30); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(pivot_size.x*30, -pivot_size.y*30); // bottom-right
		glEnd();
		glPopMatrix();
		glPopMatrix();
        for(RigidBody crate : crates){
			glPushMatrix();
			Vec bodyPosition = crate.getPosition().mul(30);
			glTranslatef(bodyPosition.x, bodyPosition.y, 0);
			glRotated(Math.toDegrees(crate.getAngle()), 0, 0, 1);
			
			//glColor3f(0.0f,0.0f,1.0f);
			crate_tex.bind();
			
			glBegin(GL_QUADS);
        
			glTexCoord2f(0, 0); glVertex2f(-size, -size); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(-size, size); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(size, size); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(size, -size); // bottom-right
        
			glEnd();

			glPopMatrix();
		
		}

    }

    public void inputListener() {
    	while (Keyboard.next()) {
    		//Keyboard
    		if (Keyboard.getEventKeyState()) {
    			switch (Keyboard.getEventKey()) {
                      case Keyboard.KEY_SPACE:
                    	  if(crateCount==0){
                    	  Vec bodyPosition = new Vec(Mouse.getX(), Mouse.getY()).mul(0.5f).mul(1 / 30f);
                    	  RigidBodyInfo crateDef = new RigidBodyInfo();
                          crateDef.position.set(bodyPosition);
                          crateDef.type = RigidBodyType.DYNAMIC;
                          
                          Polygon crateShape = new Polygon();
                          crateShape.setAsBox(crateSize, crateSize);
              
                          FixtureDef crateFixture = new FixtureDef();
                          crateFixture.density = .03f;
                          crateFixture.shape = crateShape;
                          
                          RigidBody crate = world.createBody(crateDef);
                          crate.createFixture(crateFixture);
                          
                          if(boxcreated)crateCount++;
                          crates.add(crate);
                          boxcreated = true;
                    	  }
                          break;    
                      case Keyboard.KEY_BACK:
                    	  closeDisplay();
                    	  MainMenu sim = new MainMenu(MainMenu.State.D_WHEEL);
                    	  sim.start();
                  }
              }
          }

    		if(boxcreated){
    			if (Keyboard.isKeyDown(Keyboard.KEY_S) && !Keyboard.isKeyDown(Keyboard.KEY_D)) {
    				crates.get(crateCount).applyAngularImpulse(+0.01f);
    			} else if (Keyboard.isKeyDown(Keyboard.KEY_D) && !Keyboard.isKeyDown(Keyboard.KEY_S)) {
    				crates.get(crateCount).applyAngularImpulse(-0.01f);
    			}else if (Keyboard.isKeyDown(Keyboard.KEY_A) && Mouse.isButtonDown(0)) {
    				for(RigidBody crate : crates){
    				Vec mousePosition = new Vec(Mouse.getX(),Mouse.getY()).mul(0.5f).mul(1 / 30f);
    				Vec bodyPosition = crate.getPosition();
    				Vec force = mousePosition.sub(bodyPosition);
    				crate.applyForce(force, crate.getPosition());
    				}
    			}
    			//Mouse
    			if (Mouse.isButtonDown(0)) {
    				Vec mousePosition = new Vec(Mouse.getX(),Mouse.getY()).mul(0.5f).mul(1 / 30f);
    				Vec bodyPosition = crates.get(crateCount).getPosition();
    				Vec force = mousePosition.sub(bodyPosition);
    				crates.get(crateCount).applyForce(force, crates.get(crateCount).getPosition());
    			}
    		}
    	}
    
    public void setupFloors(){

    	//pivot definition
   	 RigidBodyInfo pivotDef = new RigidBodyInfo();
        pivotDef.position.set(pivot_pos);
        pivotDef.type = RigidBodyType.KINEMATIC;
        
        //pivot shape
        Polygon pivotShape = new Polygon();
        pivotShape.setAsBox(pivot_size.x, pivot_size.y);
        FixtureDef pivotFixture = new FixtureDef();
        pivotFixture.shape = pivotShape;
        
        //create pivot
        pivot = world.createBody(pivotDef);
        pivot.createFixture(pivotFixture);
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

          RigidBodyInfo rightWallDef = new RigidBodyInfo();
          rightWallDef.position.set(((windowWidth/30)/2)-wall, 0);
          rightWallDef.type = RigidBodyType.STATIC;
          Polygon rightWallShape = new Polygon();
          rightWallShape.setAsBox(0, 1000);
          RigidBody rightWall = world.createBody(rightWallDef);
          FixtureDef rightWallFixture = new FixtureDef();
          rightWallFixture.density = 1;
          rightWallFixture.restitution = 0.3f;
          rightWallFixture.shape = rightWallShape;
          rightWall.createFixture(rightWallFixture);
          
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
          
          RigidBodyInfo platformDef = new RigidBodyInfo();
          platformDef.position.set(5.0f, 5.0f);
          platformDef.type = RigidBodyType.KINEMATIC;
          
          Polygon platformShape = new Polygon();
          platformShape.setAsBox(platformSize.x,platformSize.y);
          platform = world.createBody(groundDef);
          FixtureDef platformFix = new FixtureDef();
          platformFix.density = 1f;
          platformFix.restitution = 0.3f;
          platformFix.shape = groundShape;
          platform.createFixture(platformFix);
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

