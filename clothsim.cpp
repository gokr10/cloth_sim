
#include "clothsim.h"



//**********************************//
//      OPTIONS FOR SIMULATION      //
bool stripes = true;                //if false - shades triangles formed by the particle system different colors
bool fixedBackClothPoints = false;  //fix back corner points
bool fixedSideClothPoints = false;  //fix right side corner points
int numSpheres = 3;                 //initialize up to 3 spheres
bool userInteraction = true;        //when true, user can move the sphere(s)
//**********************************//




/*ADDITIONAL GLOBAL VARIABLES*/

//number of particles on one side of the cloth
float particleSide = 50.0f;
int numParticles = particleSide*particleSide;

//clothSide defines the length of one side of the cloth
float clothSide = 9.0f;

//The gravity vector
glm::vec3 gravity(0, -0.1, 0);
//Damping factor
float damp = .1f;

float timeStep = 1; //changes how fast and far the cloth moves by increasing acceleration
int elapsedTime = 0;
int timeEnd = 240; //simulation stopping point
float radius = 1;
bool collision = false;
bool beginSimulation = false; //start simulation by pressing 's'
bool wireframe = false; //change by pressing 'w' during simulation
float sphereTranslation = 2.0f; //sphere offset from origin when more than one sphere
float distAwayFromSpheres = 4; //cloth distance from spheres
float sphereHoriz = 0.0f; //user interaction movement of sphere initialized to 0
float sphereVert = 0.0f; //user interaction movement of sphere initialized to 0
bool beginning = true;


//The rotation variables : used to rotate the scene
float rotateX = 0;
float rotateY = 0;

//Vectors containing particle system and constraint system
std::vector<std::vector<Particle> > particleVector(particleSide);
std::vector<Constraint> constraintVector;



Particle::Particle()
{
	
}

Particle::Particle(glm::vec3 _pos)
{
  	pos = _pos;
  	mass = 1.0f;
  	oldPos = _pos;
  	accel = glm::vec3(0.0f, 0.0f, 0.0f);
  	canMove = true;
    normal = glm::vec3(0.0f, 0.0f, 0.0f);
    
}

//evaluate the force acting on each particle
void Particle::evalForce(glm::vec3 force)
{
	//for each particle, add the gravity force
	if (canMove) {
        glm::vec3 tmp;
        
        accel += force/mass;
        
        tmp = pos;

        //verlet integration
        pos = pos+(pos-oldPos)*(1.0f*damp) + accel*timeStep;
        oldPos = tmp;
        
        accel = glm::vec3(0, 0, 0);
    }
}


void Particle::sphereCollision () {
    //if distance from point to origin of sphere less than radius 2
    for (int i=0; i<numSpheres; i++) {
        glm::vec3 origin(0.0f, 0.0f, 0.0f);

        float offset = 0;
        if (i%2 == 0 && i!=0) {
            offset = -1;
        } else if (i%2 != 0 && i!=0) {
            offset = 1;
        }
            origin = origin + (glm::vec3(0, sphereVert, sphereTranslation * offset + sphereHoriz));// * glm::vec3(i, i, i));

            float dist = glm::length(pos-origin);
            
            if (dist < radius) {//-0.00001) {
                collision = true;
                
                //take pos in relation to origin at (0,0,0) to use glm::normalize
                glm::vec3 posForNorm = (pos-(glm::vec3(0, sphereVert, sphereTranslation * offset + sphereHoriz)));
                
                //push position to surface of sphere + a small number so it's not directly on surface for rendering
                pos = (glm::normalize(posForNorm) * (radius+0.01f));
                
                //re-offset so pos is set back in relation to own sphere's origin
                pos = pos + (glm::vec3(0, sphereVert, sphereTranslation * offset + sphereHoriz));
                
            }
    }

}



void Particle::changePos(glm::vec3 p)
{
	if(canMove){
		pos += p;
	}
    
}


//Freezes the particle so it can not move.
void Particle::freezeParticle()
{
	canMove = false;
}


Sphere::Sphere()
{

}



Constraint::Constraint()
{
	
}

//creates a constraint between particle 1 and particle 2
Constraint::Constraint(Particle* _part1, Particle* _part2, std::string _type) : part1(_part1), part2(_part2)
{
    
	// getting the position vector from particle 1's pos - particle 2's pos
	glm::vec3 v = part1->pos - part2->pos;
    std::string type = _type;
    
	//getting the length of the vector
	float dist = glm::length(v);
    
    if (type == "bend") {
        dist += 0.0; //CHANGE BENDINESS
    } else if ( type == "shear") {
        dist += 0.0; //CHANGE SHEAR FACTOR
    } else if (type == "structural") {
        dist += 0.0; //CHANGE STRUCTURAL FACTOR
    }
	
    //setting the structural distance / resting length of the 2 particles
	structDistance = dist;
    
}

//Ensures that the structural constraint is satisfied.
void Constraint::evalConstraint()
{
	//getting the vector from particle1 to particle2
	glm::vec3 vec12 = part2->pos - part1->pos;
	//the distance between particle1 and particle2
	float dist = glm::length(vec12);
    
    
    if (dist > structDistance) {
        //getting the difference between the particle1's distance from particle2, with the structConstraint/resting distance.
        glm::vec3 structDifference = vec12 * (1 - structDistance/dist);
        //The distance each particle must move to satisfy the the structConstraint length:
        glm::vec3 distToMove = (structDifference/2.0f);
        //Moving particle1's position to the correct resting length, structConstraint
        part1->changePos(distToMove);
        //Moving particle2's position to the correct resting length, but in the NEGATIVE direction
        part2->changePos(-distToMove);
    }
    
}

bool constraintsSatisfied() {
    
    for (int i = 0; i<constraintVector.size(); i++) {
        
        glm::vec3 vec12 = constraintVector[i].part2->pos - constraintVector[i].part1->pos;
        if (glm::length(vec12) > constraintVector[i].structDistance) {
            return false;
        }
    }
    return true;
}



ParticleSystem::ParticleSystem()
{
    
}




ParticleSystem initializeVerticalCloth(){
	ParticleSystem cloth;
	for (int y = 0; y < particleSide; y++)
	{
		for (int x = 0; x < particleSide; x++) {
			//initialize a new particle and add it to the vector
            glm::vec3 particlePos;

            particlePos = glm::vec3( (clothSide/((float) (particleSide - 1)) * x)-clothSide/2,
                                        distAwayFromSpheres,
                                        (-clothSide/((float) (particleSide - 1)) * y)+clothSide/2 );
        
			Particle currParticle(particlePos);
			particleVector[x].push_back(currParticle);
			cloth.sysPartCount += 1;
		}
        
	}

    if (fixedBackClothPoints) {
        //cloth particles to freeze -- back two points
        particleVector[particleSide-1][particleSide-1].freezeParticle();
        particleVector[particleSide-1][0].freezeParticle();
    }
    if (fixedSideClothPoints) {
        //cloth particles to freeze -- side two points
        particleVector[particleSide-1][particleSide-1].freezeParticle();
        particleVector[0][particleSide-1].freezeParticle();
    }
    
}

ParticleSystem initializeHorizCloth(){
	ParticleSystem cloth;
	for (int z = 0; z < particleSide; z++)
	{
		for (int x = 0; x < particleSide; x++) {
			//now, the y position of the cloth does not change, since it's horizontal
			glm::vec3 particlePos = glm::vec3(clothSide/((float) (particleSide - 1)) * x,
                                              0,
                                              -clothSide/((float) (particleSide - 1)) * z);
			Particle currParticle(particlePos);
			particleVector[x].push_back(currParticle);
			cloth.sysPartCount += 1;
		}
	}
}

void createConstraint(Particle* part1, Particle* part2, std::string type)
{
    
	Constraint constraint(part1, part2, type);
	constraintVector.push_back(constraint);
}

//Use this when system is full to initialize all constraints
void ParticleSystem::initializeConstraints()
{
    
    //Making constraints between directly adjacent particles (structural + shear)
    for(int y=0; y < particleSide; y++)
    {
        for(int x=0; x < particleSide; x++)
        {
            /* Structural constraints */
            if (x < particleSide-1) {
                createConstraint(&particleVector[x][y],&particleVector[x+1][y], "structural");
            }
            if (y < particleSide-1) {
                createConstraint(&particleVector[x][y],&particleVector[x][y+1], "structural");
            }
            
            /* Shear constraints */
            if (x < particleSide-1 && y < particleSide-1) {
                createConstraint(&particleVector[x][y],&particleVector[x+1][y+1], "shear");
            }
            if (x < particleSide-1 && y < particleSide-1) {
                createConstraint(&particleVector[x+1][y],&particleVector[x][y+1], "shear");
            }
            
        }
    }
    
    //Making Bend constraints
    for(int y=0; y < particleSide; y++)
    {
        for(int x=0; x < particleSide; x++)
        {
            if (x < particleSide-2) {
                createConstraint(&particleVector[x][y],&particleVector[x+2][y], "bend");
            }
            if (y < particleSide-2) {
                createConstraint(&particleVector[x][y],&particleVector[x][y+2], "bend");
            }
            if (x < particleSide-2 && y < particleSide-2) {
                createConstraint(&particleVector[x][y],&particleVector[x+2][y+2], "bend");
            }
            if (x < particleSide-2 && y < particleSide-2) {
                createConstraint(&particleVector[x+2][y],&particleVector[x][y+2], "bend");
            }
            
        }
        
    }
    
}


//Gets the normal of the triangle created by three particles PART1, PART2, PART3
glm::vec3 getTriangleNormal(Particle part1, Particle part2, Particle part3) {
	glm::vec3 v12 = part2.pos - part1.pos;
	glm::vec3 v13 = part3.pos - part1.pos;
	glm::vec3 crossProd = glm::cross(v12, v13);
	return crossProd;
    
}


void drawcloth() {
    //Draw triangles of cloth
    
    if (wireframe == true) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    
    glBegin(GL_TRIANGLES);
    glm::vec3 triNormal;
    
    //reset normals
    for (int x = 0; x<particleSide-1; x++) {
    	for (int y = 0; y<particleSide-1; y++) {
            particleVector[x][y].normal = glm::vec3(0,0,0);
        }
    }
    
    
    //calculate normals
    for (int x = 0; x<particleSide-1; x++) {
    	for (int y = 0; y<particleSide-1; y++) {
            triNormal = getTriangleNormal(particleVector[x+1][y], particleVector[x][y+1], particleVector[x][y]);
    
            //ADD NORMALS TO INDIVIDUAL PARTICLES FOR FIRST TRIANGLE
            particleVector[x+1][y].normal += glm::normalize(triNormal);
            particleVector[x][y+1].normal += glm::normalize(triNormal);
            particleVector[x][y].normal += glm::normalize(triNormal);
            
            triNormal = getTriangleNormal(particleVector[x+1][y], particleVector[x+1][y+1], particleVector[x][y+1]);
            
            //ADD NORMALS TO INDIVIDUAL PARTICLES FOR SECOND TRIANGLE
            particleVector[x+1][y].normal += glm::normalize(triNormal);
            particleVector[x+1][y+1].normal += glm::normalize(triNormal);
            particleVector[x][y+1].normal += glm::normalize(triNormal);
        }
    }
    
    //render cloth
    for (int x = 0; x<particleSide-1; x++) {
    	for (int y = 0; y<particleSide-1; y++) {
            
            if (stripes) {
                if(x%5==0){
                    glColor3f(1.0f, 0.0f, 1.0f);
                } else {
                    glColor3f(1.0f, 1.0f, 0.0f);
                }
            } else {
                glColor3f(1.0f, 0.0f, 1.0f);
            }
            //first triangle in square
            glm::normalize(particleVector[x+1][y].normal);
            glm::normalize(particleVector[x][y+1].normal);
            glm::normalize(particleVector[x][y].normal);
            
            glNormal3f(particleVector[x+1][y].normal.x, particleVector[x+1][y].normal.y, particleVector[x+1][y].normal.z); //shading
            glVertex3f(particleVector[x+1][y].pos.x, particleVector[x+1][y].pos.y, particleVector[x+1][y].pos.z);
            glNormal3f(particleVector[x][y+1].normal.x, particleVector[x][y+1].normal.y, particleVector[x][y+1].normal.z); //shading
            glVertex3f(particleVector[x][y+1].pos.x, particleVector[x][y+1].pos.y, particleVector[x][y+1].pos.z);
            glNormal3f(particleVector[x][y].normal.x, particleVector[x][y].normal.y, particleVector[x][y].normal.z); //shading
            glVertex3f(particleVector[x][y].pos.x, particleVector[x][y].pos.y, particleVector[x][y].pos.z);
            
            if (stripes) {
                if(x%5==0){
                    glColor3f(1.0f, 0.0f, 1.0f);
                } else {
                    glColor3f(1.0f, 1.0f, 0.0f);
                }
            } else {
                glColor3f(1.0f, 1.0f, 0.0f);
            }
            //second triangle in square
            glm::normalize(particleVector[x+1][y].normal);
            glm::normalize(particleVector[x+1][y+1].normal);
            glm::normalize(particleVector[x][y+1].normal);
            
            glNormal3f(particleVector[x+1][y].normal.x, particleVector[x+1][y].normal.y, particleVector[x+1][y].normal.z); //shading
            glVertex3f(particleVector[x+1][y].pos.x, particleVector[x+1][y].pos.y, particleVector[x+1][y].pos.z);
            glNormal3f(particleVector[x+1][y+1].normal.x, particleVector[x+1][y+1].normal.y, particleVector[x+1][y+1].normal.z); //shading
            glVertex3f(particleVector[x+1][y+1].pos.x, particleVector[x+1][y+1].pos.y, particleVector[x+1][y+1].pos.z);
            glNormal3f(particleVector[x][y+1].normal.x, particleVector[x][y+1].normal.y, particleVector[x][y+1].normal.z); //shading
            glVertex3f(particleVector[x][y+1].pos.x, particleVector[x][y+1].pos.y, particleVector[x][y+1].pos.z);

        }
    }
    glEnd();
    
}



//FUNCTIONS TO CREATE WINDOW AND RENDER SCENE//

class Viewport {
public:
    int w, h; // width and height
};

Viewport viewport;

void initScene(){

    GLfloat ambient[] = { 0.3, 0.3, 0.3, 0.0 };
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    GLfloat specular[] = {0.5, 0.5, 0.5, 0.0};
    GLfloat diffuse[] = {0.5, 0.5, 0.5, 0.0};
    
    GLfloat position[] = { -6.0, -6.0, 6.0, 0.0 };
    
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    
    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT1, GL_POSITION, position);

    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT2, GL_POSITION, position);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);

    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glShadeModel (GL_SMOOTH);
    
}

void myReshape(int w, int h) {
    viewport.w = w;
    viewport.h = h;
    
    glViewport (0,0,viewport.w,viewport.h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glOrtho(-6, 6, -6, 6, 6, -6);

}

void myDisplay() {
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glPushMatrix();
    glRotatef(rotateX, 0, 1, 0);
    glRotatef(rotateY, 1, 0, 0);
    
    gluLookAt(1, -0.1, -0.5,  //look from x, y, z (along z axis and with an x offset to see the cloth not strictly vertical)
              0, 0, 0,  //look at origin
              0, 1, 0); //y up vector
    

    if (elapsedTime < timeEnd) {
        
        glColor3f(0.0f, 0.0f, 1.0f);
        for (int i=0; i<numSpheres; i++) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glPushMatrix();
            float offset = 0; //offset to place spheres in different spots
            if (i%2 == 0 && i!=0) {
                offset = -1;
            } else if (i%2 != 0 && i!=0) {
                offset = 1;
            }
            glTranslatef(0.0f, sphereVert, sphereTranslation * offset + sphereHoriz); //sphereTranslation, offset for multiple spheres; sphereHoriz, sphereVert for user interaction
            glutSolidSphere(radius-.01, 25, 25); //sphere with center at origin, radius defined at top
            glPopMatrix();
        }
        
        if (beginning && !beginSimulation) { //draw once for before simulation started
            drawcloth();
        } else if (!beginSimulation) {
            drawcloth();
        }
        
        if (beginSimulation) {
            beginning = false;
            int j = 0;
            if (collision == true || fixedBackClothPoints || fixedSideClothPoints) {
                while (j<75) { //depth for evaluating constraints
                    
                    
                    for (int x=0; x<particleSide; x++) {
                        for (int y=0; y<particleSide; y++) {
                            particleVector[x][y].sphereCollision(); //check if particle collides with sphere and if so, change pos
                        }
                    }
                    
                    
                    if (collision == true || fixedBackClothPoints == true || fixedSideClothPoints == true) {
                        for (int i=0; i<constraintVector.size(); i++) {
                            constraintVector[i].evalConstraint(); //eval each Constraint in constraintVector if collision occurs
                        }
                        collision = false;
                        
                        
                        if (constraintsSatisfied()) {
                            break;
                        }
                    }
                    j++;
                    
                }
            }
            
            //if (constraintsSatisfied) {
            for (int x=0; x<particleSide; x++) {
                for (int y=0; y<particleSide; y++) {
                    particleVector[x][y].evalForce(gravity); //evalForce on particles and change positions
                }
            }
            //}
            
            for (int x=0; x<particleSide; x++) {
                for (int y=0; y<particleSide; y++) {
                    particleVector[x][y].sphereCollision(); //check if particle collides with sphere and if so, change pos
                }
            }
            drawcloth();
            elapsedTime++;
            
        }
    }
    
    glPopMatrix();
    
    //call display again if we have not reached timeEnd
    if (elapsedTime<timeEnd){
        glutPostRedisplay();
    }
    
    glFlush();
    glutSwapBuffers();
    
}


//CLOSE WINDOW WITH SPACEBAR//
void idleInput (unsigned char key, int xmouse, int ymouse) {
    switch (key)
    {
        case ' ':
            exit(0);
        case 'w':
            wireframe = !wireframe;
            break;
        case 's':
            beginSimulation = !beginSimulation;
            break;
        case 'b':
            if (beginSimulation) {
                sphereHoriz += 0.2;
            }
            break;
        case 'm':
            if (beginSimulation) {
                sphereHoriz -= 0.2;
            }
            break;
        case 'n':
            if (beginSimulation) {
                sphereVert -= 0.2;
            }
            break;
        case 'j':
            if (beginSimulation) {
                sphereVert += 0.2;
            }
            break;
        default:
            break;
    }
}

//The special keyboard functions to rotate our scene
void specialKeyFunc(int key, int x, int y) {
    
	switch(key){
		case GLUT_KEY_LEFT :
            rotateX -= 15.0f;
			break;
		case GLUT_KEY_RIGHT :
            rotateX += 15.0f;
			break;
		case GLUT_KEY_UP :
			rotateY -= 15.0f;
			break;
		case GLUT_KEY_DOWN :
			rotateY += 15.0f;
			break;
	}
    glutPostRedisplay();
}




int main(int argc, char *argv[])
{
    glutInit(&argc, argv);

	ParticleSystem cloth;
	cloth = initializeVerticalCloth();
	cloth.initializeConstraints();

    
    //CREATE WINDOW AND DRAW SCENE
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    // Initalize theviewport size
    viewport.w = 600;
    viewport.h = 600;
    glutInitWindowSize(viewport.w, viewport.h);
    glutInitWindowPosition(0,0);
    glutCreateWindow("Cloth Simulation");
    
    initScene();
    
    glutDisplayFunc(myDisplay);
    glutReshapeFunc(myReshape);
    glutKeyboardFunc(idleInput);
    glutSpecialFunc(specialKeyFunc);
    
    glutMainLoop();
    

	
	return 1;
}
