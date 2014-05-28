package frontend;

import java.io.File;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

import org.lwjgl.LWJGLException;
import org.lwjgl.input.Keyboard;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.newdawn.slick.opengl.Texture;
import org.newdawn.slick.opengl.TextureLoader;

import static org.lwjgl.opengl.GL11.*;
/**
 * Main Simulation 
 *
 */
public class MainMenu{

	//Display info
	int windowHeight = 800;
	int windowWidth = 720;
	float wallBorder = 2.4f;
	
	//Demos & Game
	demoCrate crateDemo = new demoCrate();
	demoWheel wheelDemo = new demoWheel();
	demoFriction fricDemo = new demoFriction();
	demoFlood floodDemo = new demoFlood();
	RigiBo game = new RigiBo();
	
	//Textures
	Texture background_intro;
	Texture background_mainmenu;
	Texture background_demos;
	Texture background_game;
	Texture switcher;
	
	//State of simulation menu
    public enum State {
        MAIN_MENU, 
        MM_DEMO,
        MM_GAME,
        DEMO,
        D_CRATE,
        D_WHEEL,
        D_FRICTION,
        D_FLOOD,
    }
    public State state;

    public MainMenu(State s){
    	this.state = s;
    }
    
	public void start(){
    	setupWindow();
    	setupOpenGL();
    	setupTextures();
    	open();
    }
    
    public void setupWindow(){
    	try {
            Display.setDisplayMode(new DisplayMode(windowWidth, windowHeight));
            Display.setTitle("Main Menu");
            Display.setVSyncEnabled(true);
            Display.setLocation(480,25);
            Display.create();
        } catch (LWJGLException e) {
            e.printStackTrace();
            Display.destroy();
            System.exit(1);
        }  	
    }
    
    public void setupOpenGL(){
    	 glMatrixMode(GL_PROJECTION);
    	 glOrtho(0, windowWidth/2, windowHeight/2, 0, 1, -1);
         glMatrixMode(GL_MODELVIEW);
         glEnable(GL_TEXTURE_2D);
         glEnable(GL_BLEND);
         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    public void open(){
    	//main loop
    	while (!Display.isCloseRequested()) {
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

    public void setupTextures(){
    	background_mainmenu = loadTexture("bg_mm (2)");
    	background_demos = loadTexture("bg_mm_demos (2)");
    	switcher = loadTexture("crate");
    }
    
    public void renderBackground(){
       
   
        if(state == State.MAIN_MENU 
           || state == State.MM_DEMO 
           || state == State.MM_GAME) background_mainmenu.bind();
        
        if(state == State.DEMO 
        	||state == State.D_WHEEL  
        	||state == State.D_FRICTION 
        	||state == State.D_FLOOD) background_demos.bind();

		glBegin(GL_QUADS);
		glTexCoord2f(0, 0); glVertex2f(0f, 0f); // bottom-left 
		glTexCoord2f(0, 1);glVertex2f(0.0f, windowHeight); // Upper-left
		glTexCoord2f(1, 1);glVertex2f(windowWidth, windowHeight); // Upper-right
		glTexCoord2f(1, 0);glVertex2f(windowWidth, 0.0f); // bottom-right
		glEnd();
    }
    
    public void renderSwitcher(){
    	switcher.bind();
    	glPushMatrix();
    	switch (state) {
			case MAIN_MENU:
				glTranslatef(230,220,0);
				break;
			case MM_DEMO:
				glTranslatef(230f,220f,0);
				break;
			case MM_GAME:
				glTranslatef(50f,220f,0);
				break;
			case DEMO:
				glTranslatef(50f,215,0);
				break;
			case D_WHEEL:
				glTranslatef(122f,215,0);
				break;
			case D_FRICTION:
				glTranslatef(195,215,0);
				break;
			case D_FLOOD:
				glTranslatef(270,215,0);
				break;
			default:
				break;
    	}
  
    	if(state == State.MAIN_MENU || state == State.MM_DEMO || state == State.MM_GAME){
    		glBegin(GL_QUADS);
    		glTexCoord2f(0, 0);glVertex2f(0.0f, 0.0f); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(0.0f, 5); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(90, 5); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(90, 0.0f); // bottom-right
			glEnd();
    	}
    	        
    	if(state == State.DEMO ||state == State.D_WHEEL ||
    	  state == State.D_FRICTION ||state == State.D_FLOOD){
    		glBegin(GL_QUADS);
			glTexCoord2f(0, 0);glVertex2f(0.0f, 0.0f); // bottom-left 
			glTexCoord2f(0, 1);glVertex2f(0.0f, 5); // Upper-left
			glTexCoord2f(1, 1);glVertex2f(50, 5); // Upper-right
			glTexCoord2f(1, 0);glVertex2f(50, 0.0f); // bottom-right
			glEnd();
    	}
    	glPopMatrix();
    }
    public void renderDisplay() {
        glClear(GL_COLOR_BUFFER_BIT);
        renderBackground();
        renderSwitcher();
    }

    public void inputListener() {

    	while (Keyboard.next()) {
    		switch (state) {
            	case MAIN_MENU:
            		if (Keyboard.isKeyDown(Keyboard.KEY_RIGHT)) state = State.MM_DEMO;
            		if (Keyboard.isKeyDown(Keyboard.KEY_LEFT)) state = State.MM_GAME;
            		if (Keyboard.isKeyDown(Keyboard.KEY_RETURN)) state = State.DEMO;
            		if (Keyboard.isKeyDown(Keyboard.KEY_ESCAPE)) exit();
            		break;
                case MM_DEMO:
                	if (Keyboard.isKeyDown(Keyboard.KEY_RETURN)) state = State.DEMO;
                	if(Keyboard.isKeyDown(Keyboard.KEY_LEFT)) state = State.MM_GAME;
                	if (Keyboard.isKeyDown(Keyboard.KEY_ESCAPE)) exit();
                	break;
                case MM_GAME:
                    if (Keyboard.isKeyDown(Keyboard.KEY_RETURN)){
                    	closeDisplay();
                    	RigiBo r = new RigiBo();
                    	r.start();
                    }
                    if(Keyboard.isKeyDown(Keyboard.KEY_RIGHT)) state = State.MM_DEMO;
                	if (Keyboard.isKeyDown(Keyboard.KEY_ESCAPE)) exit();
                    break;
                case DEMO:
                	if (Keyboard.isKeyDown(Keyboard.KEY_RETURN)){
                		closeDisplay();
                		crateDemo.start();
                	}
                	if (Keyboard.isKeyDown(Keyboard.KEY_RIGHT))state = State.D_WHEEL;
                	if (Keyboard.isKeyDown(Keyboard.KEY_BACK))state = State.MAIN_MENU;
                	if (Keyboard.isKeyDown(Keyboard.KEY_ESCAPE)) exit();
                    break;
                case D_CRATE:
                	if (Keyboard.isKeyDown(Keyboard.KEY_BACK)) state = State.DEMO;
                	if (Keyboard.isKeyDown(Keyboard.KEY_ESCAPE)) exit();
                	break;
                case D_WHEEL:
                	if (Keyboard.isKeyDown(Keyboard.KEY_RETURN)){
                		closeDisplay();
                		wheelDemo.start();
                	}
                	if (Keyboard.isKeyDown(Keyboard.KEY_RIGHT))state = State.D_FRICTION;
                	if (Keyboard.isKeyDown(Keyboard.KEY_LEFT))state = State.DEMO;
                	if (Keyboard.isKeyDown(Keyboard.KEY_BACK))state = State.MAIN_MENU;
                	if (Keyboard.isKeyDown(Keyboard.KEY_ESCAPE)) exit();
                	break;
                case D_FRICTION:
                	if (Keyboard.isKeyDown(Keyboard.KEY_RETURN)){
                		closeDisplay();
                		fricDemo.start();
                	}
                	if (Keyboard.isKeyDown(Keyboard.KEY_RIGHT))state = State.D_FLOOD;
                	if (Keyboard.isKeyDown(Keyboard.KEY_LEFT))state = State.D_WHEEL;
                	if (Keyboard.isKeyDown(Keyboard.KEY_BACK)) state = State.MAIN_MENU;
                	if (Keyboard.isKeyDown(Keyboard.KEY_ESCAPE)) exit();
                	break;
                case D_FLOOD:
                	if (Keyboard.isKeyDown(Keyboard.KEY_RETURN)){
                		closeDisplay();
                		floodDemo.start();
                	}
                   	if (Keyboard.isKeyDown(Keyboard.KEY_LEFT))state = State.D_FRICTION;
                	if (Keyboard.isKeyDown(Keyboard.KEY_BACK)) state = State.DEMO;
                	if (Keyboard.isKeyDown(Keyboard.KEY_ESCAPE)) exit();
                	break;
                default:
                	break;
    			}
    		}	
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
    public static void main(String[] args) {
		MainMenu sim = new MainMenu(MainMenu.State.MAIN_MENU);
		sim.start();
	}
	}