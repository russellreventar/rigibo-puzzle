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
import physics.collision.ContactListener;
import physics.collision.shapes.Circle;
import physics.collision.shapes.Polygon;
import physics.dynamics.Fixture;
import physics.dynamics.RigidBody;
import physics.dynamics.RigidBodyInfo;
import physics.dynamics.RigidBodyType;
import physics.dynamics.FixtureDef;
import physics.dynamics.Pool;
import physics.dynamics.contacts.Contact;
import physics.tools.Vec;


public class RigiBo3{
	 float fade = 0f;
	//display info
	int windowHeight = 800;
	int windowWidth = 720;
	float wallBorder = 2.4f;

	//BALL DATA
	RigidBody Mball;
	Vec Mball_pos =new Vec(3f,5.2f+.1f);
	float ball_radius = .45f;
	float ball_bounce = .2f;
	float ball_mass = .3f;
	Vec MStand_pos = new Vec(3f,5.2f);
	//BOX DATA
	Vec box_size = new Vec(toCord((windowWidth / 5)),.2f);
	float box_bounce = 0.1f;
	float box_mass = 0.3f;
	Vec topBall_pos = new Vec(.5f,11.25f);
	Vec domoBall_pos = new Vec(1f,13f);
	//TRIANGLE DATA
	float tri_height = 1f;
	float tri_base = 1f;
	
	//RECTANGLE 
	Vec rect_size = new Vec(1.3f,.2f); 
	ContactListener my;
	Contact s;
	
	//2ND FLOOR
	Vec floor2_size = new Vec(4f,.2f);
	Vec floor2_pos = new Vec(5f,11f);

	//1ST FLOOR
	Vec floor1_size = new Vec(3f,.2f);
	Vec floor1_pos = new Vec(6f,7.5f);
	
	Vec wall_size = new Vec(.2f,5.2f);
	Vec wall_pos = new Vec(1.2f,6f);
	
	Vec floorStart_size = new Vec(.5f,.2f);
	Vec floorStart_pos = new Vec(1.5f,12f);
	
	//DOMINOES
	Vec domo_size = new Vec(.2f, .75f);
	Vec domo_pos = new Vec(3f, floor2_pos.y + floor2_size.y + domo_size.y);
	float domo_mass = .3f;
	float domo_bounce = .1f;
	
	RigidBody trampoline;
	Vec tramp_pos = new Vec(.5f,1.3f);
	Vec tramp_size = new Vec(.5f,.5f);
	
	Vec tramp2_pos = new Vec(1.9f,1.3f);
	Vec tramp2_size = new Vec(.5f,.5f);
	
	float spacing = .45f;


	Pool world = new Pool(new Vec(0.0f,-9.8f),true); //World space
	
	
	
	
	ArrayList<RigidBody> crates = new ArrayList<RigidBody>(); //Crates
	ArrayList<RigidBody> triangles = new ArrayList<RigidBody>();
	ArrayList<RigidBody> rectangles = new ArrayList<RigidBody>();
	ArrayList<RigidBody> balls = new ArrayList<RigidBody>();
	ArrayList<RigidBody> tramps = new ArrayList<RigidBody>();
	float floorX = (windowWidth/5) + (windowWidth/5)/2;
	Vec floorPos = new Vec(toCord(windowWidth)/2,4.0f);
	Vec floorSize = new Vec(toCord(floorX),0.25f);
	RigidBody floor;
	int waitTime = 1;
	float finWaitTime = 4.5f;
	float floorWin = floorPos.y + floorSize.y + ball_radius + .5f;
	float dt = 1/80f; //delta time
	float timer = 0;
	
	float aVel = 4f;
	Vec box1_pos = new Vec(floorPos.x + floorPos.x/2 + spacing*3 + .2f , floorPos.y +floorSize.y + 3.5f);
	
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
	Texture tex_rect;
	Texture tex_static;
	Texture tex_tramp;
	
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
  
  public enum elevator{
	  UP, DOWN
  }
  public State boxState = State.LISTEN;
  public GameState gameState =GameState.RUNNING;
  public elevator elev = elevator.UP;

  public void setupTextures(){
  	tex_crate = loadTexture("cube");
  	tex_background = loadTexture("bg");
  	tex_floor = loadTexture("crate");
  	tex_ball = loadTexture("Main_ball");
  	tex_red = loadTexture("wait1");
	tex_green = loadTexture("wait3");
	tex_check = loadTexture("check");
	tex_miss = loadTexture("miss");
	tex_boxOpen = loadTexture("open");
	tex_ballRed = loadTexture("ball_red");
	tex_rect = loadTexture("rect");
	tex_static = loadTexture("static-01");
	tex_tramp = loadTexture("tramp");
  }
  
  public void renderDisplay() {

  		glClear(GL_COLOR_BUFFER_BIT);
  		glLoadIdentity();
     
  		renderBackground();
  		renderFloor();
  		renderBoxes();
  		renderBall();
  		renderTriangles();
  		renderRectangles();
  		if(boxState == State.TIMER) renderWaitClock();
    	if(boxState == State.ONFLOOR)renderFinClock();
    	if(boxState == State.COMPLETE)renderCheckmark();
    	
  	  for(RigidBody crate : tramps){
  			glPushMatrix();
  			Vec bodyPosition = crate.getPosition().mul(30);
  			glTranslatef(bodyPosition.x, bodyPosition.y, 0);
  			glRotated(Math.toDegrees(crate.getAngle()), 0, 0, 1);

  			Fixture shapeFix = crate.getFixtureList();
  			Polygon shape = (Polygon) shapeFix.getShape();

  			tex_tramp.bind();
  			glBegin(GL_QUADS);
  			glTexCoord2f(0, 0);glVertex2f(-shape.m_vertices[0].x*30, -shape.m_vertices[0].y*30); // bottom-left 
  			glTexCoord2f(0, 1);glVertex2f(-shape.m_vertices[0].x*30, shape.m_vertices[0].y*30); // Upper-left
  			glTexCoord2f(1, 1);glVertex2f(shape.m_vertices[0].x*30, shape.m_vertices[0].y*30); // Upper-right
  			glTexCoord2f(1, 0);glVertex2f(shape.m_vertices[0].x*30, -shape.m_vertices[0].y*30); // bottom-right
  			glEnd();

  			glPopMatrix();
  		}

  }

  public void renderRectangles(){
	  for(RigidBody crate : rectangles){
			glPushMatrix();
			Vec bodyPosition = crate.getPosition().mul(30);
			glTranslatef(bodyPosition.x, bodyPosition.y, 0);
			glRotated(Math.toDegrees(crate.getAngle()), 0, 0, 1);

			Fixture shapeFix = crate.getFixtureList();
			Polygon shape = (Polygon) shapeFix.getShape();

			tex_rect.bind();
			glBegin(GL_QUADS);
			glTexCoord2f(0, 0);glVertex2f(-shape.m_vertices[0].x*30, -shape.m_vertices[0].y*30); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(-shape.m_vertices[0].x*30, shape.m_vertices[0].y*30); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(shape.m_vertices[0].x*30, shape.m_vertices[0].y*30); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(shape.m_vertices[0].x*30, -shape.m_vertices[0].y*30); // bottom-right
			glEnd();

			glPopMatrix();
		}
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

			if(crate.getType() == RigidBodyType.STATIC){
				tex_static.bind();
			}else{
			tex_crate.bind();
			}
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
	  				Fixture shapeFix = crate.getFixtureList();
	  				Polygon shape = (Polygon) shapeFix.getShape();
	  				float hx =  Math.abs(shape.m_vertices[0].x);
	  				float hy =Math.abs(shape.m_vertices[0].y);
	  				
	  				if(mousePos.x > (crate.getPosition().x - hx) && 
	  							mousePos.x < (crate.getPosition().x + hx) &&
	  							mousePos.y < (crate.getPosition().y + hy) &&
	  							mousePos.y > (crate.getPosition().y - hy)){
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
                  	  gameState =GameState.RUNNING;
                    case Keyboard.KEY_2:
                    	closeDisplay();
                    	RigiBo r = new RigiBo();
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
  
  private RigidBody createRect(Vec size, Vec pos, float d, float r,RigidBodyType rt){
	   	
  	  RigidBodyInfo bodyDef = new RigidBodyInfo();
      bodyDef.position.set(pos);
      bodyDef.type = rt;
      Polygon boxShape = new Polygon();
      boxShape.setAsBox(size.x,size.y);
      RigidBody body = world.createBody(bodyDef);
      
      FixtureDef boxFixture = new FixtureDef();
      boxFixture.density = d;
      boxFixture.restitution = r;
      boxFixture.shape = boxShape;
      
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
	  
	  for(RigidBody ball:rectangles){
		  world.destroyBody(ball);
	  }
	  triangles = new ArrayList<RigidBody>();
	  crates = new ArrayList<RigidBody>();
	  balls = new ArrayList<RigidBody>();
	  rectangles = new ArrayList<RigidBody>();
  }
  
  
  public void createBodies(){
	  	  
	  rectangles.add(createRect(rect_size, box1_pos, box_mass,box_bounce, RigidBodyType.KINEMATIC));
	  rectangles.add(createRect(floor2_size, floor2_pos, box_mass,box_bounce, RigidBodyType.STATIC));
	  rectangles.add(createRect(floorStart_size, floorStart_pos, box_mass,box_bounce, RigidBodyType.STATIC));
	  tramps.add(createRect(tramp_size, tramp_pos, box_mass,1.2f, RigidBodyType.STATIC));
	  tramps.add(createRect(tramp2_size, tramp2_pos, box_mass,1.2f, RigidBodyType.STATIC));
	  
	  Vec stand = new Vec(floorStart_size.y, floorStart_size.x);
	  Vec stand2 = new Vec(floor2_pos.x + floor2_size.x -.2f, floor2_pos.y + floor2_size.y + stand.y);
	  rectangles.add(createRect(stand, stand2, box_mass,1.2f, RigidBodyType.STATIC));
	  
	  for(float i = 0; i <= 5 ; i++){
		  Vec pos = new Vec(domo_pos.x + i, domo_pos.y);
		  crates.add(createRect(domo_size, pos, domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  }
 
	  crates.add(createRect(new Vec(.9f,.9f), new Vec(10.5f,11f), domo_mass, domo_bounce, RigidBodyType.STATIC));
	  
	  
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(4.3f,7f), domo_mass, domo_bounce, RigidBodyType.STATIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(4.3f,8f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(4.3f,9f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(4.3f,10), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(5.4f,7f), domo_mass, domo_bounce, RigidBodyType.STATIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(5.4f,8f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(5.4f,9f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(5.4f,10f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));

	  crates.add(createRect(new Vec(.5f,.5f), new Vec(6.5f,7f), domo_mass, domo_bounce, RigidBodyType.STATIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(6.5f,8f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(6.5f,9f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(6.5f,10f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(7.6f,7f), domo_mass, domo_bounce, RigidBodyType.STATIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(7.6f,8f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(7.6f,9f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createRect(new Vec(.5f,.5f), new Vec(7.6f,10f), domo_mass, domo_bounce, RigidBodyType.DYNAMIC));
	  crates.add(createRect(new Vec(.4f,.4f), new Vec(.5f,10.8f), domo_mass, domo_bounce, RigidBodyType.STATIC));
	 
	  rectangles.add(createRect(new Vec(.05f,.1f), MStand_pos, 0.0f,0.0f,RigidBodyType.STATIC));
	  rectangles.add(createRect(wall_size, wall_pos, box_mass,box_bounce, RigidBodyType.STATIC));

	 
	  //BALLS
	 
	  Mball = createBall(ball_radius, Mball_pos, ball_mass,ball_bounce);
	  balls.add(createBall(ball_radius, topBall_pos, ball_mass,ball_bounce));
	  balls.add(createBall(ball_radius, domoBall_pos, ball_mass+.2f,ball_bounce));
	  balls.add(createBall(ball_radius, new Vec(stand2.x,stand2.y + stand.y + ball_radius), ball_mass - .2f,ball_bounce));

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
          Display.setTitle("RigiBo Puzzle Level 3");
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

	  rectangles.get(0).setAngularVelocity(aVel);
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

