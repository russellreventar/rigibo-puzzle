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

public class demoFriction{


	int windowHeight = 1000;
	int windowWidth = 1000;
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
	Texture tex_box;
	Texture tex_ice;
	Texture tex_conc;
	Texture tex_background;
	
	RigidBody floor_ice;
	Vec ice_pos = new Vec(7.0f,10f);
	Vec ice_size = new Vec(7f,0.25f);
	
	RigidBody floor_conc;
	Vec conc_pos = new Vec(7.0f,2.0f);
	Vec conc_size = new Vec(7f,0.25f);
	
	RigidBody body_ice;
	Vec bodyice_pos = new Vec(7.0f,11.0f);
	Vec bodyice_size = new Vec(1.5f,1.5f);
	
	RigidBody body_conc;
	Vec bodyconc_pos = new Vec(7.0f,3.0f);
	Vec bodyconc_size = new Vec(1.5f,1.5f);

    private enum State {
    	ICE, CONCRETE
    }
    private State block = State.ICE;

    public void start(){
    	setupWindow();
    	initializeGL();
    	initializeTextures();
    	open();
    }
    
    public void setupWindow(){
        
    	try {
            Display.setDisplayMode(new DisplayMode(windowWidth, windowHeight));
            Display.setInitialBackground(200.0f, 200.0f, 200.0f);
            Display.setTitle("Friction Demo");
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
    	//crate_tex = loadTexture("crate");
    	while (!Display.isCloseRequested()) {
        	
        	if(Keyboard.isKeyDown(Keyboard.KEY_ESCAPE))exit();
    		doStep();
        	renderDisplay();
        	inputListener();
            updateDisplay();
        }
    	exit();
    }

    public void initializeTextures(){
    	//tex_background = loadTexture("bg_friction");
    	tex_box = loadTexture("crate");
    	tex_ice = loadTexture("Ice");
    	tex_background = loadTexture("bg");
    	tex_conc = loadTexture("crate");
    }
    public void updateDisplay(){
    	Display.update();
    	Display.sync(40);
    }
    
    public void closeWindow(){
    	Display.destroy();
    }
    
    public void exit(){
    	Display.destroy();
    	System.exit(0);
    }
    public void doStep() {
        world.step(dt, 8, 3);
    }
    public void renderDisplay() {
        //if(crateCount > 0){
    	glClear(GL_COLOR_BUFFER_BIT);

//    	renderBackground();
//    	renderFloors();
//    	renderBoxes();
    	glPushMatrix();
    	tex_background.bind();
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(0f, 0f); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(0.0f, windowHeight); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(windowWidth, windowHeight); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(windowWidth, 0.0f); // bottom-right
		glEnd();
    	glPopMatrix();
    	
    	glPushMatrix();
		glTranslatef(floor_ice.getPosition().x * 30, floor_ice.getPosition().y * 30, 0);
		glRotated(Math.toDegrees(floor_ice.getAngle()), 0, 0, 1);
		tex_ice.bind();
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(-(ice_size.x)*30, -ice_size.y*30); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(-ice_size.x*30, ice_size.y*30); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(ice_size.x*30, ice_size.y*30); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(ice_size.x*30, -ice_size.y*30); // bottom-right
		glEnd();
		glPopMatrix();
		
		glPushMatrix();
		glTranslatef(floor_conc.getPosition().x*30, floor_conc.getPosition().y*30, 0);
		glRotated(Math.toDegrees(floor_conc.getAngle()), 0, 0, 1);
		
		tex_conc.bind();
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(-(conc_size.x)*30, -conc_size.y*30); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(-conc_size.x*30, conc_size.y*30); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(conc_size.x*30, conc_size.y*30); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(conc_size.x*30, -conc_size.y*30); // bottom-right
		glEnd();
		glPopMatrix();
        
		glPushMatrix();
		glTranslatef(body_ice.getPosition().x*30, body_ice.getPosition().y*30, 0);
		glRotated(Math.toDegrees(body_ice.getAngle()), 0, 0, 1);
		
		tex_box.bind();
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(-bodyice_size.x*30, -bodyice_size.y*30); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(-bodyice_size.x*30, bodyice_size.y*30); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(bodyice_size.x*30, bodyice_size.y*30); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(bodyice_size.x*30, -bodyice_size.y*30); // bottom-right
		glEnd();
		glPopMatrix();
		
		glPushMatrix();
		glTranslatef(body_conc.getPosition().x*30, body_conc.getPosition().y*30, 0);
		glRotated(Math.toDegrees(body_conc.getAngle()), 0, 0, 1);
		
		tex_box.bind();
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(-bodyconc_size.x*30, -bodyconc_size.y*30); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(-bodyconc_size.x*30, bodyconc_size.y*30); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(bodyconc_size.x*30, bodyconc_size.y*30); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(bodyconc_size.x*30, -bodyconc_size.y*30); // bottom-right
		glEnd();

		glPopMatrix();
    }
    
    public void renderBackground(){
    	
    }
    
    public void renderFloors(){
    	//tex_ice.bind();
    	glPushMatrix();
    	glTranslatef(floor_ice.getPosition().x * 30, floor_ice.getPosition().y * 30, 0);
    	glRotated(Math.toDegrees(floor_ice.getAngle()), 0, 0, 1);
	
    	glColor3f(0.0f,0.0f,1.0f);
    	glBegin(GL_QUADS);
    	glTexCoord2f(0, 0); glVertex2f(-(ice_size.x)*60, -ice_size.y*60); // bottom-left 
    	glTexCoord2f(0, 1);glVertex2f(-ice_size.x*30, ice_size.y*30); // Upper-left
    	glTexCoord2f(1, 1);glVertex2f(ice_size.x*30, ice_size.y*30); // Upper-right
    	glTexCoord2f(1, 0);glVertex2f(ice_size.x*30, -ice_size.y*30); // bottom-right
    	glEnd();
    	glPopMatrix();
    }
    
    public void renderBoxes(){
    	
    }

    public void inputListener() {
    	while (Keyboard.next()) {
    		//Keyboard
    		if (Keyboard.getEventKeyState()) {
    			switch (Keyboard.getEventKey()) {
                	case Keyboard.KEY_SPACE:
                		switch(block){
          		    		case ICE: block = State.CONCRETE;break;
          		    		case CONCRETE: block = State.ICE;break;
          		    	}break;
                	case Keyboard.KEY_BACK:
                		closeWindow();
                    	MainMenu sim = new MainMenu(MainMenu.State.D_FRICTION);
                    	sim.start();
                    	break;
    			}
    		}
    	}

    	if (Keyboard.isKeyDown(Keyboard.KEY_S) && !Keyboard.isKeyDown(Keyboard.KEY_D)) {
    		switch(block){
    			case ICE: body_ice.applyAngularImpulse(0.01f); break;
    			case CONCRETE: body_conc.applyAngularImpulse(0.01f); break;
    		}
    	}else if (Keyboard.isKeyDown(Keyboard.KEY_D) && !Keyboard.isKeyDown(Keyboard.KEY_S)) {
    		switch(block){
    			case ICE: body_ice.applyAngularImpulse(-0.01f);break;
    			case CONCRETE: body_conc.applyAngularImpulse(-0.01f); break;
    		}
    	}
    	
    	if (Keyboard.isKeyDown(Keyboard.KEY_A) && Mouse.isButtonDown(0)) {
    		Vec mousePosition = new Vec(Mouse.getX(),Mouse.getY()).mul(0.5f).mul(1 / 30f);
    		body_ice.applyForce(mousePosition.sub(body_ice.getPosition()), body_ice.getPosition());
    		body_conc.applyForce(mousePosition.sub(body_conc.getPosition()), body_conc.getPosition());
    	}
    	
    	//Mouse
    	if (Mouse.isButtonDown(0)) {
    		Vec mousePosition = new Vec(Mouse.getX(),Mouse.getY()).mul(0.5f).mul(1 / 30f);
  			switch(block){
    	    	case ICE: body_ice.applyForce(mousePosition.sub(body_ice.getPosition()), body_ice.getPosition()); break;
    	    	case CONCRETE: body_conc.applyForce(mousePosition.sub(body_conc.getPosition()), body_conc.getPosition()); break;
  			}
    	}		
    }
    
    public void setupFloors(){

    	//ice definition
   	 	RigidBodyInfo pivotDef = new RigidBodyInfo();
   	 	pivotDef.position.set(ice_pos);
        pivotDef.type = RigidBodyType.STATIC;
      
        //ice shape
        Polygon pivotShape = new Polygon();
        pivotShape.setAsBox(ice_size.x, ice_size.y);
        FixtureDef iceFixture = new FixtureDef();
        iceFixture.friction = 0.0f;
        iceFixture.shape = pivotShape;

        
        //create ice
        floor_ice = world.createBody(pivotDef);
        floor_ice.createFixture(iceFixture);

          
      	//conc definition
        RigidBodyInfo concDef = new RigidBodyInfo();
        concDef.position.set(conc_pos);
        concDef.type = RigidBodyType.STATIC;
             
        //conc shape
        Polygon concShape = new Polygon();
        concShape.setAsBox(ice_size.x, ice_size.y);
        FixtureDef concFixture = new FixtureDef();
        concFixture.friction = 7.0f;
        concFixture.shape = concShape;

         floor_conc = world.createBody(concDef);
         floor_conc.createFixture(concFixture);
          
   	  RigidBodyInfo bodyiceDef = new RigidBodyInfo();
      bodyiceDef.position.set(bodyice_pos);
      bodyiceDef.type = RigidBodyType.DYNAMIC;
      
      RigidBodyInfo bodyconcDef = new RigidBodyInfo();
      bodyconcDef.position.set(bodyconc_pos);
      bodyconcDef.type = RigidBodyType.DYNAMIC;
      
      Polygon crateShape = new Polygon();
      crateShape.setAsBox(crateSize, crateSize);

      FixtureDef bodyiceFixture = new FixtureDef();
      bodyiceFixture.density = .03f;
      bodyiceFixture.shape = crateShape;
      
      FixtureDef bodyconcFixture = new FixtureDef();
      bodyconcFixture.density = .03f;
      bodyconcFixture.shape = crateShape;
      
      body_ice = world.createBody(bodyiceDef);
      body_ice.createFixture(bodyiceFixture);
      
      body_conc = world.createBody(bodyconcDef);
      body_conc.createFixture(bodyconcFixture);
     
         
         
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

