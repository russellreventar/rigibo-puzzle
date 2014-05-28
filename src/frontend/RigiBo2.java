package frontend;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.io.File;
import org.lwjgl.LWJGLException;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.newdawn.slick.opengl.Texture;
import org.newdawn.slick.opengl.TextureLoader;

import static org.lwjgl.opengl.GL11.*;
import physics.collision.shapes.Circle;
import physics.collision.shapes.Polygon;
import physics.dynamics.Fixture;
import physics.dynamics.RigidBody;
import physics.dynamics.RigidBodyInfo;
import physics.dynamics.RigidBodyType;
import physics.dynamics.FixtureDef;
import physics.dynamics.Pool;
import physics.tools.Vec;

public class RigiBo2{
	 float fade = 0f;
	//display info
	int windowHeight = 800;
	int windowWidth = 720;
	float wallBorder = 2.4f;

	//BALL DATA
	RigidBody Mball;
	float ball_radius = .45f;
	float ball_bounce = .4f;
	float ball_mass = .3f;
	
	//BOX DATA
	Vec box_size = new Vec(toCord((windowWidth / 5)),.2f);
	float box_bounce = 0.1f;
	float box_mass = 0.3f;
	
	//TRIANGLE DATA
	float tri_height = 1f;
	float tri_base = 1f;
	
	float spacing = .45f;
	Pool world = new Pool(new Vec(0.0f,-9.8f),true); //World space
	ArrayList<RigidBody> crates = new ArrayList<RigidBody>(); //Crates
	ArrayList<RigidBody> triangles = new ArrayList<RigidBody>();
	ArrayList<RigidBody> balls = new ArrayList<RigidBody>();
	float floorX = (windowWidth/5) + (windowWidth/5)/2;
	Vec floorPos = new Vec(toCord(windowWidth)/2,4.0f);
	Vec floorSize = new Vec(toCord(floorX),0.25f);
	RigidBody floor;
	int waitTime = 1;
	float finWaitTime = 4.5f;
	float floorWin = floorPos.y + floorSize.y + ball_radius + .5f;
	float dt = 1/80f; //delta time
	float timer = 0;
	
	Vec clock_Pos = new Vec(toCord(windowWidth)/2, floorPos.y - 2f);
	float clock_size = 1f;
	//TEXTURES
	Texture tex_crate;
	Texture tex_background;
	Texture tex_floor;
	Texture tex_ball;
	Texture tex_miss;
	Texture tex_red;
	Texture tex_check;
	Texture tex_green;
	Texture tex_boxOpen;
	Texture tex_ballRed;
	
	int toDelete = 0;
	
	//State of simulation 
  public enum GameState {
  	RUNNING,PAUSE,RESET
  }
  
  public enum waitState{
	  green,yellow,red
  }
  
  public enum State{
	  LISTEN, PREDELETE, DELETE, TIMER, ONFLOOR, COMPLETE
  }
  public State boxState = State.LISTEN;
  public GameState gameState =GameState.RUNNING;

  public void setupTextures(){
  	tex_crate = loadTexture("loo-01");
  	tex_background = loadTexture("bg3");
  	tex_floor = loadTexture("crate");
  	tex_ball = loadTexture("Main_ball");
  	tex_red = loadTexture("wait1");
	tex_green = loadTexture("wait3");
	tex_check = loadTexture("check");
	tex_miss = loadTexture("miss");
	tex_boxOpen = loadTexture("open2");
	tex_ballRed = loadTexture("ball_red");
	
  }
  
  public void renderDisplay() {

  		glClear(GL_COLOR_BUFFER_BIT);
  		glLoadIdentity();
     
  		renderBackground();
  		renderFloor();
  		renderBoxes();
  		renderBall();
  		renderTriangles();
  		if(boxState == State.TIMER) renderWaitClock();
    	if(boxState == State.ONFLOOR)renderFinClock();
    	if(boxState == State.COMPLETE)renderCheckmark();
    	

  }

  public void renderCheckmark(){
	  	glPushMatrix();
	  	glTranslatef(clock_Pos.x*30, clock_Pos.y*30, 0);
		  tex_check.bind();
		  glBegin(GL_QUADS);
			glTexCoord2f(0, 0);glVertex2f(-clock_size*30, -clock_size*30); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(-clock_size*30, clock_size*30); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(clock_size*30, clock_size*30); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(clock_size*30, -clock_size*30); // bottom-right
			glEnd();
			glPopMatrix();
	  
  }
  	public void renderWaitClock(){
	  	glPushMatrix();
	  	glTranslatef(clock_Pos.x*30, clock_Pos.y*30, 0);
		  tex_red.bind();
		  glBegin(GL_QUADS);
			glTexCoord2f(0, 0);glVertex2f(-clock_size*30, -clock_size*30); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(-clock_size*30, clock_size*30); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(clock_size*30, clock_size*30); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(clock_size*30, -clock_size*30); // bottom-right
			glEnd();
			glPopMatrix();
  }
  
  	public void renderFinClock(){
  		glPushMatrix();
  		float interval = finWaitTime/3;
  		tex_green.bind();
  		if(timer < finWaitTime){
  			glPushMatrix();
  			glTranslatef((clock_Pos.x - clock_size*2 - .10f)*30, clock_Pos.y*30, 0);
  			glBegin(GL_QUADS);
			glTexCoord2f(0, 0);glVertex2f(-clock_size*30, -clock_size*30); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(-clock_size*30, clock_size*30); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(clock_size*30, clock_size*30); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(clock_size*30, -clock_size*30); // bottom-right
			glEnd();
			glPopMatrix();	
  		}
  		if(timer < finWaitTime && timer > interval){
  			glPushMatrix();
  			glTranslatef(clock_Pos.x*30, clock_Pos.y*30, 0);
  			glBegin(GL_QUADS);
			glTexCoord2f(0, 0);glVertex2f(-clock_size*30, -clock_size*30); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(-clock_size*30, clock_size*30); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(clock_size*30, clock_size*30); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(clock_size*30, -clock_size*30); // bottom-right
			glEnd();
			glPopMatrix();
	  }
	  if(timer < finWaitTime && timer >= interval*2 - .4f){
		  	glPushMatrix();
		  	glTranslatef((clock_Pos.x + clock_size*2 + .10f)*30, clock_Pos.y*30, 0);
		  	glBegin(GL_QUADS);
			glTexCoord2f(0, 0);glVertex2f(-clock_size*30, -clock_size*30); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(-clock_size*30, clock_size*30); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(clock_size*30, clock_size*30); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(clock_size*30, -clock_size*30); // bottom-right
			glEnd();
			glPopMatrix();
	  }
	  glPopMatrix();
 
  }
  public void renderBoxes(){
	   for(RigidBody crate : crates){
 			glPushMatrix();
 			Vec bodyPosition = crate.getPosition().mul(30);
 			glTranslatef(bodyPosition.x, bodyPosition.y, 0);
 			glRotated(Math.toDegrees(crate.getAngle()), 0, 0, 1);

 			Fixture shapeFix = crate.getFixtureList();
 			Polygon shape = (Polygon) shapeFix.getShape();

 			tex_crate.bind();
 			glBegin(GL_QUADS);
 			glTexCoord2f(0, 0);glVertex2f(-shape.m_vertices[0].x*30, -shape.m_vertices[0].y*30); // bottom-left 
 			glTexCoord2f(0, 1);glVertex2f(-shape.m_vertices[0].x*30, shape.m_vertices[0].y*30); // Upper-left
 			glTexCoord2f(1, 1);glVertex2f(shape.m_vertices[0].x*30, shape.m_vertices[0].y*30); // Upper-right
 			glTexCoord2f(1, 0);glVertex2f(shape.m_vertices[0].x*30, -shape.m_vertices[0].y*30); // bottom-right
 			glEnd();

 			glPopMatrix();
			}
  }
  
  public void renderTriangles(){

	   for(RigidBody tri : triangles){
				glPushMatrix();
				Vec bodyPosition = tri.getPosition().mul(30);
				glTranslatef(bodyPosition.x, bodyPosition.y, 0);
				glRotated(Math.toDegrees(tri.getAngle()), 0, 0, 1);
				tex_boxOpen.bind();
				glBegin(GL_TRIANGLES);
				glTexCoord2f(0, 1);glVertex2f(0*30, tri_height*30); // bottom-left 
				glTexCoord2f(1, -1);glVertex2f(-tri_base*30, -tri_height*30); // Upper-left
				glTexCoord2f(-1, -1);glVertex2f(tri_base*30, -tri_height*30); // Upper-right
				
				glEnd();

				glPopMatrix();
			}
  }
  
  public void renderBall(){

	   for(RigidBody ball : balls){
				glPushMatrix();
				Vec bodyPosition = ball.getPosition().mul(30);
				glTranslatef(bodyPosition.x, bodyPosition.y, 0);
				glRotated(Math.toDegrees(ball.getAngle()), 0, 0, 1);
				tex_ballRed.bind();
				glBegin(GL_QUADS);
				glTexCoord2f(0, 0);glVertex2f(-ball_radius*30, -ball_radius*30); // bottom-left 
				glTexCoord2f(0, 1);glVertex2f(-ball_radius*30, ball_radius*30); // Upper-left
				glTexCoord2f(1, 1);glVertex2f(ball_radius*30, ball_radius*30); // Upper-right
				glTexCoord2f(1, 0);glVertex2f(ball_radius*30, -ball_radius*30); // bottom-right
				glEnd();
				glPopMatrix();
			}
	   
	  	glPushMatrix();
		Vec bodyPosition = Mball.getPosition().mul(30);
		glTranslatef(bodyPosition.x, bodyPosition.y, 0);
		glRotated(Math.toDegrees(Mball.getAngle()), 0, 0, 1);
		
		tex_ball.bind();
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0);glVertex2f(-ball_radius*30, -ball_radius*30); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(-ball_radius*30, ball_radius*30); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(ball_radius*30, ball_radius*30); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(ball_radius*30, -ball_radius*30); // bottom-right
		glEnd();
		glPopMatrix();
  }
  public void renderFloor(){
	  glPushMatrix();
	  glTranslatef(floorPos.x*30, floorPos.y*30, 0);
	  tex_floor.bind();
	  glBegin(GL_QUADS);
	  glTexCoord2f(0, 0);glVertex2f(-floorSize.x*30, -floorSize.y*30); // bottom-left 
	  glTexCoord2f(0, 1);glVertex2f(-floorSize.x*30, floorSize.y*30); // Upper-left
	  glTexCoord2f(1, 1);glVertex2f(floorSize.x*30, floorSize.y*30); // Upper-right
	  glTexCoord2f(1, 0);glVertex2f(floorSize.x*30, -floorSize.y*30); // bottom-right
	  glEnd();
	  glPopMatrix();
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
	  if(Mball.getPosition().y < 0){
		  gameState = GameState.RESET;
	  }
	  switch(boxState){
	  	case LISTEN:
	  		if (Mouse.isButtonDown(0)) {
	  			
	  			Vec mousePos = new Vec(Mouse.getX(),Mouse.getY()).mul(0.5f).mul(1 / 30f);
	  			for(RigidBody crate:crates){
	  				if(mousePos.x > (crate.getPosition().x - box_size.x) && 
	  							mousePos.x < (crate.getPosition().x + box_size.x) &&
	  							mousePos.y < (crate.getPosition().y + box_size.y) &&
	  							mousePos.y > (crate.getPosition().y - box_size.y)){
	  					world.destroyBody(crate);
	  					crates.remove(crate);
	  					boxState = State.TIMER;
	  					break;
	  				}
	  			}
	  			
	  			for(RigidBody tri:triangles){
	  				if(mousePos.x > (tri.getPosition().x - tri_base) && 
	  							mousePos.x < (tri.getPosition().x + tri_base) &&
	  							mousePos.y < (tri.getPosition().y + tri_height) &&
	  							mousePos.y > (tri.getPosition().y - tri_height)){
	  					world.destroyBody(tri);
	  					
	  					triangles.remove(tri);	
	  					boxState = State.TIMER;
	  					break;
	  				}
	  			}
	  			
	  			for(RigidBody ball:balls){
	  				if(mousePos.x > (ball.getPosition().x - ball_radius) && 
	  							mousePos.x < (ball.getPosition().x + ball_radius) &&
	  							mousePos.y < (ball.getPosition().y + ball_radius) &&
	  							mousePos.y > (ball.getPosition().y - ball_radius)){
	  					world.destroyBody(ball);
	  					balls.remove(ball);	
	  					boxState = State.TIMER;
	  					break;
	  				}
	  			}
	  		}
	  		if(Mball.getPosition().y - ball_radius < floorPos.y  + ball_radius && Mball.getPosition().x > floorPos.x - floorSize.x && Mball.getPosition().x < floorPos.x+floorSize.x){
	  		  boxState = State.ONFLOOR;
	  		  }
	  		break;
	  	case TIMER:
	  		timer+=dt;
	  		if(timer > waitTime){
	  			timer = 0;
	  			boxState = State.LISTEN;
	  			break;
	  		}
	  		if(Mball.getPosition().y - ball_radius < floorPos.y +  ball_radius && Mball.getPosition().x > floorPos.x - floorSize.x && Mball.getPosition().x < floorPos.x+floorSize.x){
		  		  boxState = State.ONFLOOR;
		  		  }
	  		break;
	  	case ONFLOOR:
	  		timer+=dt;
	  		if(timer >= finWaitTime-1f){
	  			if(Mball.getPosition().y < floorWin){
	  				timer = 0;
	  				boxState = State.COMPLETE;
	  				break;
	  			}else{
	  				timer = 0;
	  				boxState = State.LISTEN;
	  				break;
	  			}
	  		}
	  		break;
	  	default:
	  		break;

	  }

  	while (Keyboard.next()) {
  		if (Keyboard.getEventKeyState()) {
  			switch (Keyboard.getEventKey()) {
                    
                    //return to simulation menu
                    case Keyboard.KEY_BACK:
                  	  closeDisplay();
                  	  MainMenu sim = new MainMenu(MainMenu.State.MAIN_MENU);
                  	  sim.start();
                  	  break;
                  	  
                  	//Pause Simulation
                    case Keyboard.KEY_P:
                    	if(gameState == GameState.RUNNING){
                    		gameState =GameState.PAUSE;
                    	}else if(gameState == GameState.PAUSE){
                    		gameState = GameState.RUNNING;
                    	}
                  	 break;
                    case Keyboard.KEY_R:
                    	boxState = State.LISTEN;
                    	gameState = GameState.RESET;
                    	break;
                    //Resume simulation
                    case Keyboard.KEY_RETURN:
                  	  if(boxState == State.COMPLETE){
                  		  closeDisplay();
                  		  RigiBo3 rp = new RigiBo3();
                  		  rp.start();
                  	  }else
                    	gameState =GameState.RUNNING;
                  	  break;
                    case Keyboard.KEY_2:
                    	closeDisplay();
                    	RigiBo3 r = new RigiBo3();
                    	r.start();
  			}	
  		}
  		
  	}
			
  }
  private RigidBody createBall(float radius, Vec pos, float d, float r){
	   	
	  RigidBodyInfo ballDef = new RigidBodyInfo();
      ballDef.position.set(pos);
      ballDef.type = RigidBodyType.DYNAMIC;
 	
      Circle c = new Circle();
      c.m_radius = radius;
      
      FixtureDef ballFixture = new FixtureDef();
      ballFixture.density = d;
      ballFixture.restitution = r;
      ballFixture.shape = c;

      RigidBody ball = world.createBody(ballDef);
      ball.createFixture(ballFixture);

      
      return ball;
  }
  private RigidBody createBox(Vec pos, float d, float r,RigidBodyType rt){
	   	
  	  RigidBodyInfo bodyDef = new RigidBodyInfo();
      bodyDef.position.set(pos);
      bodyDef.type = rt;
      Polygon boxShape = new Polygon();
      boxShape.setAsBox(box_size.x,box_size.y);
      RigidBody body = world.createBody(bodyDef);
      
      FixtureDef boxFixture = new FixtureDef();
      boxFixture.density = d;
      boxFixture.restitution = r;
      boxFixture.shape = boxShape;
      
      body.createFixture(boxFixture);   
      
      return body;
  }
  
  	private RigidBody createTriangle(Vec pos, float d, float r){
	   	Vec vertices[] = new Vec[3];
	 	vertices[2] = new Vec(0,tri_height);
	 	vertices[1] = new Vec(tri_base,-tri_height);
	 	vertices[0] = new Vec(-tri_base,-tri_height);
	 	RigidBodyInfo bodyDef = new RigidBodyInfo();
	 	bodyDef.position.set(pos);
	 	bodyDef.type = RigidBodyType.DYNAMIC;
	 	bodyDef.angle = 0f;
	 	Polygon boxShape = new Polygon();
	 	boxShape.set(vertices, 3);

	 	RigidBody body = world.createBody(bodyDef);
    
	 	FixtureDef boxFixture = new FixtureDef();
	 	boxFixture.density = d;
	  	boxFixture.restitution = r;
	  	boxFixture.shape = boxShape;
	  	boxFixture.friction=0.6f;
	  	body.createFixture(boxFixture);   
    
	  	return body;
  	}
  public void destroyBodies(){
	
	  world.destroyBody(Mball);
	  for(RigidBody crate:crates){
		  world.destroyBody(crate);
	  }
	  for(RigidBody tri:triangles){
		  world.destroyBody(tri);
	  }
	  for(RigidBody ball:balls){
		  world.destroyBody(ball);
	  }
	  triangles = new ArrayList<RigidBody>();
	  crates = new ArrayList<RigidBody>();
	  balls = new ArrayList<RigidBody>();
  }
  
  
  public void createBodies(){
	  
	  //BOXES
	  Vec box1_pos = new Vec(floorPos.x - floorPos.x/2 + spacing , floorPos.y +floorSize.y + tri_height*4 + box_size.y);
	  Vec box2_pos = new Vec(floorPos.x - floorPos.x/2 + spacing*2 + box_size.x - spacing , 12f - ball_radius - box_size.y);
	  
	  crates.add(createBox(box1_pos, box_mass,box_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createBox(box2_pos, box_mass,box_bounce, RigidBodyType.STATIC));
	  
	  //TRIANGLES
	  Vec tri_pos1 = new Vec(floorPos.x - floorPos.x/2 + spacing , floorPos.y +floorSize.y + tri_height);
	  Vec tri_pos2 = new Vec(floorPos.x - floorPos.x/2 + spacing , floorPos.y +floorSize.y + tri_height*3);
	  triangles.add(createTriangle(tri_pos1, 0.1f,0.2f));
	  triangles.add(createTriangle(tri_pos2, 0.1f,0.2f));
	  //BALLS
	  Vec ball_pos1 = new Vec(floorPos.x - floorPos.x/2 + spacing + box_size.x - spacing,floorPos.y +floorSize.y + tri_height*4 + box_size.y*2 + ball_radius);
	  Vec ball_pos2 = new Vec(floorPos.x - floorPos.x/2 + spacing*2 + box_size.x - spacing,12f);
	  //MBALL
	  Vec Mball_pos = new Vec(floorPos.x - floorPos.x/2 + spacing - box_size.x + spacing,floorPos.y +floorSize.y + tri_height*4 + box_size.y*2 + ball_radius);
	  Mball = createBall(ball_radius, Mball_pos, ball_mass,ball_bounce);
	  

	  
	  balls.add(createBall(ball_radius, ball_pos1, ball_mass,ball_bounce));
	  balls.add(createBall(ball_radius, ball_pos2, ball_mass,ball_bounce));

  }
  	public void setupFloors(){

	  	//Left Wall
  	
  		Polygon wallShape = new Polygon();
  		wallShape.setAsBox(0, windowHeight);

  		FixtureDef wallFixture = new FixtureDef();
  		wallFixture.density = 1;
  		wallFixture.restitution = 0.3f;
  		wallFixture.shape = wallShape;
  	
  		//Define body info
      	RigidBodyInfo leftWallDef = new RigidBodyInfo();
  		leftWallDef.position.set(0, 0);
  		leftWallDef.type = RigidBodyType.STATIC;
  		//Define body info
      	RigidBodyInfo rightWallDef = new RigidBodyInfo();
  		leftWallDef.position.set(toCord(windowWidth), 0);
  		leftWallDef.type = RigidBodyType.STATIC;

  		RigidBody leftWall = world.createBody(leftWallDef);
  		leftWall.createFixture(wallFixture);

        //create
  		RigidBody rightWall = world.createBody(rightWallDef);
  		rightWall.createFixture(wallFixture);
   
        
  		//Ground
  		RigidBodyInfo groundDef = new RigidBodyInfo();
  		groundDef.position.set(floorPos);
  		groundDef.type = RigidBodyType.STATIC;
     
  		Polygon groundShape = new Polygon();
  		groundShape.setAsBox(floorSize.x,floorSize.y);
      
  		FixtureDef groundFixture = new FixtureDef();
  		groundFixture.density = 1f;
     	groundFixture.restitution = 0.3f;
     	groundFixture.shape = groundShape;
     
     	RigidBody floor = world.createBody(groundDef);
     	floor.createFixture(groundFixture);
  }
  public void setupDisplay(){
  	try {
          Display.setDisplayMode(new DisplayMode(windowWidth, windowHeight));
          Display.setInitialBackground(200.0f, 200.0f, 200.0f);
          Display.setTitle("RigiBo Puzzle Level 2");
          Display.setLocation(480,25);
          Display.setVSyncEnabled(true);
          Display.create();
      } catch (LWJGLException e) {
          e.printStackTrace();
          Display.destroy();
          System.exit(1);
      }  	
  }
  
  public float toPix(float a){
	 float p = a * 30 * 2;
	 return p;
  }
  
  public float toCord(float a){
		 float p = (a / 30) / 2;
		 return p;
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
	  switch(gameState){
	  	case RUNNING:
	  		world.step(dt, 8, 3);
	  		break;
	  	case RESET:
	  		destroyBodies();
	  		createBodies();
	  		gameState = GameState.RUNNING;
	  	break;
	  	default:
		break;
	  }
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
	  createBodies();
  	while (!Display.isCloseRequested()) {
      	if(Keyboard.isKeyDown(Keyboard.KEY_ESCAPE))exit();

      		doStep();
      		inputListener();
      		renderDisplay();
      		updateDisplay();
          
      }
  	releaseTextures();
  	exit();
  }
  public void releaseTextures(){
	  tex_crate.release();
	  tex_background.release();
	  tex_floor.release();
  }
  public void setupOpenGL(){
  	 glMatrixMode(GL_PROJECTION);
  	 glOrtho(0, windowWidth/2, 0, windowHeight/2, 1, -1);
     glMatrixMode(GL_MODELVIEW);
    // glClearColor(0,0,0,1);
     glEnable(GL_TEXTURE_2D);
     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
     glEnable( GL_BLEND );
  }
  
  public Texture loadTexture(String key){
  	try {
			return TextureLoader.getTexture("PNG", new FileInputStream(new File("resources/game/" + key + ".png")));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
  	return null;
  }
}

