package frontend;

//Utilities
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.io.File;

//Game Libraries
import org.lwjgl.LWJGLException;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.newdawn.slick.opengl.Texture;
import org.newdawn.slick.opengl.TextureLoader;

import static org.lwjgl.opengl.GL11.*;

//Physics Libraries
import physics.collision.shapes.Polygon;
import physics.dynamics.RigidBody;
import physics.dynamics.RigidBodyInfo;
import physics.dynamics.RigidBodyType;
import physics.dynamics.FixtureDef;
import physics.dynamics.Pool;
import physics.tools.Vec;


public class demoCrate{
	 float fade = 0f;
	//display info
	int windowHeight = 800;
	int windowWidth = 720;
	float wallBorder = 0f;
	float wall = 0f;

	float crateSize = 2.3f; //size of crate
	boolean boxcreated = false; //initial box flag
	
	Pool world = new Pool(new Vec(0.0f,-9.8f),true); //World space
	ArrayList<RigidBody> crates = new ArrayList<RigidBody>(); //Crates
	
	int crateCount = 0; //number of crates created
	float dt = 1/60f; //delta time
	
	//textures
	Texture tex_crate;
	Texture tex_background;
	
	//State of simulation 
    public enum State {
    	RUNNING,PAUSE
    }
    public State state = State.RUNNING;

    public void setupTextures(){
    	tex_crate = loadTexture("crate");
    	tex_background = loadTexture("bg_crate");
    }
    
    public void renderDisplay() {
        //if(crateCount > 0){
    	glClear(GL_COLOR_BUFFER_BIT);
        float size = (crateSize)*30;
		
        renderBackground();
 
       	//crates
        for(RigidBody crate : crates){
			glPushMatrix();
			Vec bodyPosition = crate.getPosition().mul(30);
			glTranslatef(bodyPosition.x, bodyPosition.y, 0);
			glRotated(Math.toDegrees(crate.getAngle()), 0, 0, 1);
			
			
			tex_crate.bind();
			glBegin(GL_QUADS);
			glTexCoord2f(0, 0);glVertex2f(-size, -size); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(-size, size); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(size, size); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(size, -size); // bottom-right
			glEnd();

			glPopMatrix();
		}

    }

    public void renderBackground(){
    	glPushMatrix();
    	tex_background.bind();
    	glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(0.0f, 0.0f); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(0.0f, windowHeight); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(windowWidth, windowHeight); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(windowWidth, 0.0f); // bottom-right
		glEnd();
		glPopMatrix();
    }
    public void inputListener() {

    	//Listen for keyboard inputs
    	while (Keyboard.next()) {
    		if (Keyboard.getEventKeyState()) {
    			switch (Keyboard.getEventKey()) {

    				  //create new box
                      case Keyboard.KEY_SPACE:
                    	  Vec bodyPosition = new Vec(Mouse.getX(), Mouse.getY()).mul(0.5f).mul(1 / 30f);
                    	  RigidBodyInfo crateDef = new RigidBodyInfo();
                          crateDef.position.set(bodyPosition);
                          crateDef.type = RigidBodyType.DYNAMIC;
                          
                          Polygon crateShape = new Polygon();
                          crateShape.setAsBox(crateSize, crateSize);
              
                          FixtureDef crateFixture = new FixtureDef();
                          crateFixture.density = .03f;
                          crateFixture.shape = crateShape;
                          
                          RigidBody newcrate = world.createBody(crateDef);
                          newcrate.createFixture(crateFixture);
                          
                          if(boxcreated)crateCount++;
                          crates.add(newcrate);
                          boxcreated = true;
                          break;    
                      
                      //return to simulation menu
                      case Keyboard.KEY_BACK:
                    	  closeDisplay();
                    	  MainMenu sim = new MainMenu(MainMenu.State.DEMO);
                    	  sim.start();
                    	  break;
                    	  
                      //Pause Simulation
                      case Keyboard.KEY_P:
                    	  state = State.PAUSE;
                    	 break;
                    
                      //Resume simulation
                      case Keyboard.KEY_RETURN:
                    	  state = State.RUNNING;
    			}	
    		}
    	}
    	
    	if(boxcreated){
   
    		//Rotate 
    		if (Keyboard.isKeyDown(Keyboard.KEY_S) && !Keyboard.isKeyDown(Keyboard.KEY_D)) {
    				crates.get(crateCount).applyAngularImpulse(+0.01f);
    			} else if (Keyboard.isKeyDown(Keyboard.KEY_D) && !Keyboard.isKeyDown(Keyboard.KEY_S)) {
    				crates.get(crateCount).applyAngularImpulse(-0.01f);
    		
    		//control All boxes
    		}else if (Keyboard.isKeyDown(Keyboard.KEY_A) && Mouse.isButtonDown(0)) {
    				for(RigidBody crate : crates){
    				Vec mousePosition = new Vec(Mouse.getX(),Mouse.getY()).mul(0.5f).mul(1 / 30f);
    				Vec bodyPosition = crate.getPosition();
    				Vec force = mousePosition.sub(bodyPosition);
    				crate.applyForce(force, crate.getPosition());
    				}
    			}
    		
    		//Control current box
    		if (Mouse.isButtonDown(0)) {
    			Vec mousePosition = new Vec(Mouse.getX(),Mouse.getY()).mul(0.5f).mul(1 / 30f);
    			Vec bodyPosition = crates.get(crateCount).getPosition();
    			Vec force = mousePosition.sub(bodyPosition);
    			crates.get(crateCount).applyForce(force, crates.get(crateCount).getPosition());
    		}
    	}
    }
    
    public void setupFloors(){

    	//Left Wall
    	
    	//Define body info
    	RigidBodyInfo leftWallDef = new RigidBodyInfo();
    	leftWallDef.position.set(wall, 0);
    	leftWallDef.type = RigidBodyType.STATIC;
        
    	//Wall shape
    	Polygon leftWallShape = new Polygon();
        leftWallShape.setAsBox(0, 1000);
      
        //Wall attributes
        FixtureDef leftWallFixture = new FixtureDef();
        leftWallFixture.density = 1;
        leftWallFixture.restitution = 0.3f;
        leftWallFixture.shape = leftWallShape;
        
        //create
        RigidBody leftWall = world.createBody(leftWallDef);
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
          
        //Ground
       RigidBodyInfo groundDef = new RigidBodyInfo();
       groundDef.position.set(0, wall);
       groundDef.type = RigidBodyType.STATIC;
       
       Polygon groundShape = new Polygon();
       groundShape.setAsBox(1000, 0);
        
       FixtureDef groundFixture = new FixtureDef();
       groundFixture.density = 1f;
       groundFixture.restitution = 0.3f;
       groundFixture.shape = groundShape;
       
       RigidBody ground = world.createBody(groundDef);
       ground.createFixture(groundFixture);
    }
    public void setupDisplay(){
    	try {
            Display.setDisplayMode(new DisplayMode(windowWidth, windowHeight));
            Display.setInitialBackground(200.0f, 200.0f, 200.0f);
            Display.setTitle("Crate Demo");
            Display.setLocation(480,25);
            Display.setVSyncEnabled(true);
            Display.create();
        } catch (LWJGLException e) {
            e.printStackTrace();
            Display.destroy();
            System.exit(1);
        }  	
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
        world.step(dt, 8, 3);
    }
    public void start(){
    	setupDisplay();
    	setupOpenGL();
    	setupTextures();
    	setupFloors();
    	open();
    }
    
    public void open(){

    	//Main simulation loop
    	while (!Display.isCloseRequested()) {
        	
        	if(Keyboard.isKeyDown(Keyboard.KEY_ESCAPE))exit();
        	
        	if(state == State.RUNNING) doStep();
        	inputListener();
        	renderDisplay();
            updateDisplay();
            
        }
    	exit();
    }
    
    public void setupOpenGL(){
    	 glMatrixMode(GL_PROJECTION);
    	 glOrtho(0, windowWidth/2, 0, windowHeight/2, 1, -1);
         glMatrixMode(GL_MODELVIEW);
         glEnable(GL_TEXTURE_2D);
         glEnable(GL_BLEND);
         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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

