#/** * * * BME-VIK-MI_2018_02_08 * * *|4.HF| * * * * * * * * * *\
#*    _ _____   _____        __ _                              *
#*   (_)  __ \ / ____|      / _| |                             *
#*   |_| |__)| (___    ___ | |_| |___      ____ _ _ __ ___     *
#*   | |  _  / \___ \ / _ \|  _| __\ \ /\ / / _` | '__/ _ \    *
#*   | | | \ \ ____) | (_) | | | |_ \ V  V / (_| | | |  __/    *
#*   |_|_|  \_\_____/ \___/|_|  \__| \_/\_/ \__,_|_|  \___|    *
#*                                                             *
#*                   http://irsoftware.net                     *
#*                                                             *
#*              contact_adress: sk8Geri@gmail.com               *
#*                                                               *
#*       This file is a part of the work done by aFagylaltos.     *
#*         You are free to use the code in any way you like,      *
#*         modified, unmodified or copied into your own work.     *
#*        However, I would like you to consider the following:    *
#*                                                               *
#*  -If you use this file and its contents unmodified,         *
#*              or use a major part of this file,               *
#*     please credit the author and leave this note untouched.   *
#*  -If you want to use anything in this file commercially,      *
#*                please request my approval.                    *
#*                                                              *
#\* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <math.h>
#include <stdlib.h>

#if defined(__APPLE__)
#include <OpenGL/gl.h>                                                                                                                                                                                                            
#include <OpenGL/glu.h>                                                                                                                                                                                                           
#include <GLUT/glut.h>                                                                                                                                                                                                            
#else
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#include <windows.h>                                                                                                                                                                                                              
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#endif

#define PI       3.14159265358979323846
const int screenWidth = 600;
const int screenHeight = 600;
const int N = 30;
const float g = 0.1f;
const float ballRadius = .2;
const float headRadius = .3;

#define LOT_OF_BALL
//#define _DEBUG    // for debugging skeleton

#ifdef _DEBUG
GLUquadric *qobj;
#endif

enum {
    PLANE,
    SPHERE,
    EYE,
    BALL,
    SEAL
};

struct Vector {
    float x, y, z, h;

    Vector() {
        x = y = z = 1;
        h = 0.0f;
    }

    Vector(float x0, float y0, float z0, float h0 = 1) {
        x = x0;
        y = y0;
        z = z0;
        h = h0;
    }

    float &operator[](int i) { return *(&x + i); }

    Vector operator*(float a) const {
        return Vector(x * a, y * a, z * a, h * a);
    }

    Vector operator/(float d) const {
        return Vector(x / d, y / d, z / d, h / d);
    }

    Vector operator+(const Vector &v) const {
        return Vector(x + v.x, y + v.y, z + v.z, h + v.h);
    }

    Vector operator-(const Vector &v) const {
        return Vector(x - v.x, y - v.y, z - v.z, h - v.h);
    }

    float operator*(const Vector &v) const {     // dot product
        return (x * v.x + y * v.y + z * v.z + h * v.h);
    }

    Vector operator%(const Vector &v) {
        return Vector(y * v.z - v.y * z, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    float Length() { return sqrtf(x * x + y * y + z * z + h * h); }

    Vector &Normalize() {
        float norm = Length();
        x /= norm;
        y /= norm;
        z /= norm;
        h /= norm;
        return *this;
    }
};

Vector headPosAvg;
Vector headPos;
Vector ballPos;
Vector sealPos;
Vector eye0;
Vector eye1;

Vector rotateVector(Vector in, Vector center, float ax, float ay, float dx, float dy) {
    Vector v = in;
    v = v - center;

    // cos -sin
    // sin  cos
    v = Vector(v.x, v.y * cos(-ay) + v.z * sin(-ay), v.y * -sin(-ay) + v.z * cos(-ay));
    v = Vector(v.x * cos(-ax) + v.z * sin(-ax), v.y, v.x * -sin(-ax) + v.z * cos(-ax));

    v = Vector(v.x * cos(dx + ax) + v.z * sin(dx + ax), v.y, v.x * -sin(dx + ax) + v.z * cos(dx + ax));
    v = Vector(v.x, v.y * cos(dy + ay) + v.z * sin(dy + ay), v.y * -sin(dy + ay) + v.z * cos(dy + ay));

    v = v + center;

    return v;
}

float randFloat(float a, float b) {
    return ((b - a) * ((float) rand() / RAND_MAX)) + a;
}

class Bone {
public:
    Bone() {
    }

    void make(Vector _position, float _l, float _angleX, float _angleY, float _angleZ) {
        position = _position;
        l = _l;
        angleX = _angleX;
        angleY = _angleY;
        angleZ = _angleZ;
    }

    void draw(float t) {
        glTranslatef(position.x, position.y, position.z);
        glutSolidSphere(0.1, 16, 16);
        glRotatef(-angleX * 180 / PI, 0, 1, 0);
        glRotatef(-angleY * 180 / PI, 1, 0, 0);
        glRotatef(angleZ * 180 / PI, 0, 0, 1);
#ifdef _DEBUG
        gluCylinder(qobj, 0.1, 0.01, l, 32, 4);
#endif
        glTranslatef(0, 0, l);
    }

public:
    Vector position;
    Vector center;
    float l;
    float angleX;
    float angleY;
    float angleZ;
} head, tail;

struct Color {
    float r, g, b, a;

    Color() {
        r = g = b = 0;
        a = 1;
    }

    Color(float r0, float g0, float b0, float a0 = 1) {
        r = r0;
        g = g0;
        b = b0;
        a = a0;
    }

    Color operator*(float a) {
        return Color(r * a, g * a, b * a);
    }

    Color operator*(const Color &c) {
        return Color(r * c.r, g * c.g, b * c.b);
    }

    Color operator+(const Color &c) {
        return Color(r + c.r, g + c.g, b + c.b);
    }

    Color operator-(const Color &c) {
        return Color(r - c.r, g - c.g, b - c.b);
    }
};

class Texture {
    GLuint texID;
public:
    Texture(int w, int h, Color *texture) {
        glGenTextures(1, &texID);
        glBindTexture(GL_TEXTURE_2D, texID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGBA, GL_FLOAT, texture);
    }

    void apply() {
        glBindTexture(GL_TEXTURE_2D, texID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
};

struct Material {
    float s;
    Color ka, kd, ks;

    Material(Color _ka, Color _kd, Color _ks, float _s) {
        ka = _ka;
        kd = _kd;
        ks = _ks;
        s = _s;
    }

    void apply() {
        glMaterialfv(GL_FRONT, GL_AMBIENT, &ka.r);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, &kd.r);
        glMaterialfv(GL_FRONT, GL_SPECULAR, &ks.r);
        glMaterialf(GL_FRONT, GL_SHININESS, s);
    }
};

class Object {
    GLuint type;
protected:
    Vector position;
    Vector axis;
    float angle;
    Vector scale;

public:

    Object() {
        axis = Vector(0, 0, 0);
        angle = 0;
    }

    void setType(GLuint _type) {
        type = _type;
    }

    GLuint getType() {
        return type;
    }

    Vector getPosition() {
        return position;
    }

    void transalte(Vector _newPos) {
        position = _newPos;
    }

    void rotate(float _angle, Vector _axis) {
        angle = _angle;
        axis = _axis;
    }

    void scalef(Vector _scale) {
        scale = _scale;
    }

    void begin() {
        glPushMatrix();
        glScalef(scale[0], scale[1], scale[2]);
        glTranslatef(position.x, position.y, position.z);
        glRotatef(angle, axis[0], axis[1], axis[2]);
    }

    virtual void draw(float time) = 0;

    void end() {
        glPopMatrix();
    }
};

class ParametizedMesh : public Object {
    Material *material;
    Texture *texture;
public:

    void make(GLuint _type, Vector _position, Material *_material, Texture *_texture = NULL) {
        setType(_type);
        position = _position;
        material = _material;
        texture = _texture;
    }

    virtual void vertex(float u, float v) = 0;

    void draw(float time) {

        if (material) {
            glEnable(GL_LIGHTING);
            material->apply();
        } else {
            glDisable(GL_LIGHTING);
        }

        if (texture) {
            glEnable(GL_TEXTURE_2D);
            texture->apply();

            if (material)
                glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            else
                glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        } else {
            glDisable(GL_TEXTURE_2D);
        }

#ifdef _DEBUG
        glBegin(GL_LINES);
#else
        glBegin(GL_QUADS);
#endif
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                vertex((float) i / N, (float) j / N);
                vertex((float) (i) / N, (float) (j + 1) / N);
                vertex((float) (i + 1) / N, (float) (j + 1) / N);
                vertex((float) (i + 1) / N, (float) (j) / N);
            }
        }
        glEnd();
    }
};

class Plane : public ParametizedMesh {
    Vector points[4];
    Vector normal;
public:
    Plane(Vector _points[]) {
        for (int i = 0; i < 4; i++)
            points[i] = _points[i];

        normal = (points[1] - points[0]) % (points[2] - points[0]);
    }

    void vertex(float u, float v) {
        glTexCoord2f(u, v);

        Vector vx = (points[0] * (1 - u) +
                     points[1] * u) * (1 - v) +

                    (points[3] * (1 - u) +
                     points[2] * u) * v;

        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(vx.x, vx.y, vx.z);
    }

    Vector getPoint(int index) {
        return points[index];
    }
};

class Sphere : public ParametizedMesh {
    float radius;
public:
    Sphere(float _radius = 1) {
        radius = _radius;
    }

    void vertex(float u, float v) {
        glTexCoord2f(u, v);

        float theta = u * 2 * PI;
        float phi = v * PI;
        Vector vx = Vector(radius * cos(theta) * sin(phi),
                           radius * sin(theta) * sin(phi),
                           radius * cos(phi));


        glNormal3f(vx.x, vx.y, vx.z);
        glVertex3f(vx.x, vx.y, vx.z);
    }
};

class Ball : public ParametizedMesh {
    float radius;
public:
    Vector acc;

    Ball(float _radius = 1) {
        radius = _radius;
        acc = Vector(0, 0, 0);
    }

    void force(float f) {
        acc = acc * f;
    }

    void vertex(float u, float v) {
        glTexCoord2f(u, v);

        float theta = u * 2 * PI;
        float phi = v * PI;
        Vector vx = Vector(radius * cos(theta) * sin(phi),
                           radius * sin(theta) * sin(phi),
                           radius * cos(phi));

        glNormal3f(vx.x, vx.y, vx.z);
        glVertex3f(vx.x, vx.y, vx.z);
    }
};

class BezierCurve {
    Vector p[9];
    int numPoints;

    float B(int i, float t) {
        float choose = 1;

        for (int j = 1; j <= i; j++) {
            choose *= (float) (numPoints - j + 1) / j;
        }

        return choose * pow(t, i) * pow(1 - t, numPoints - i);
    }


    float derivateB(int i, float t) {
        float choose = 1;

        for (int j = 1; j <= i; j++) {
            choose *= (float) (numPoints - j + 1) / j;
        }

        float fdg = i * pow(t, i - 1) * pow(1 - t, numPoints - i);
        float gdf = pow(t, i) * (numPoints - i) * pow(1 - t, numPoints - i - 1) * (-1);

        if (i == 0) {
            return gdf * choose;
        } else if (i == numPoints) {
            return fdg * choose;
        }

        return (fdg + gdf) * choose;
    }

public:

    int getNumPoint() {
        return numPoints;
    }

    Vector getPoint(int index) {
        return p[index];
    }

    void setPoint(int index, Vector v) {
        p[index] = v;
    }

    BezierCurve() {
        numPoints = -1;
    }

    BezierCurve(Vector points[], int num) {
        for (int i = 0; i < num; i++) {
            p[i] = Vector(points[i].x, points[i].y, points[i].z);
        }
        numPoints = num - 1;
    }

    void addControlPoint(Vector point) {
        p[++numPoints] = point;
    }


    Vector rd(float t) {
        Vector rr(0, 0, 0);

        for (int i = 0; i <= numPoints; i++) {
            rr = rr + p[i] * derivateB(i, t);
        }

        return rr;
    }

    Vector r(float t) {
        Vector rr(0, 0, 0);

        for (int i = 0; i <= numPoints; i++) {
            rr = rr + p[i] * B(i, t);
        }

        return rr;
    }
};

class SealBody : public ParametizedMesh {
    BezierCurve sealNode[7];
    int numNode;

public:

    void moveHead(float dx, float dy) {

        head.angleX += dx;
        head.angleY += dy;

        //eye0 = rotateVector(eye0,Vector(-0.2, 0.4, -0.7)+Vector(-1,-3.3,3.8),-head.angleX,-head.angleY,-dy,-dx);

        Vector summa = Vector(0, 0, 0);
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 8; j++) {
                Vector v = rotateVector(sealNode[i].getPoint(j), head.center, head.angleX, head.angleY, dx, dy);
                summa = summa + v;
                sealNode[i].setPoint(j, v);
            }
        }
        headPosAvg = summa / 16;
    }

    void addNode(BezierCurve node) {
        sealNode[numNode++] = node;
    }

    SealBody() {
        numNode = 0;
    }

    void vertex(float u, float v) {
        glTexCoord2f(u, v);

        BezierCurve node;
        for (int i = 0; i < numNode; i++) {
            node.addControlPoint(sealNode[i].r(u));
        }

        Vector vd = node.rd(v);
        BezierCurve node2;
        for (int i = 0; i < numNode; i++) {
            node2.addControlPoint(sealNode[i].rd(u));
        }

        Vector ud = node2.r(v);
        Vector vx = node.r(v);
        Vector n = (ud % vd);

        glNormal3fv((GLfloat *) &n);
        glVertex3f(vx.x, vx.y, vx.z);
    }
};

class Seal : public Object {
    SealBody body;
    SealBody fin;
    Sphere eyes[2];

public:
    Seal(GLuint _type, Vector _position) {
        setType(_type);
        position = Vector(0, 0, 0);

        float x0, y0, z;
        float x1, y1;
        float x2, y2;
        float x3, y3;

        //----orr----------------------------
        x0 = 0;
        y0 = 0.65;
        x1 = 0.1;
        y1 = 0.7;
        x2 = 0.14;
        y2 = 0.83;
        x3 = 0.06;
        y3 = 0.93;
        z = 0;

        Vector p1[] = {Vector(x0, y0, z), Vector(x1, y1, z), Vector(x2, y2, z), Vector(x3, y3, z),
                       Vector(-x3, y3, z), Vector(-x2, y2, z), Vector(-x1, y1, z), Vector(x0, y0, z)};

        body.addNode(BezierCurve(p1, 8));

        //----fej----------------------------
        x0 = 0;
        y0 = 0;
        x1 = 0.63;
        y1 = 0.28;
        x2 = 0.79;
        y2 = 0.97;
        x3 = 0.35;
        y3 = 1.53;
        z = 0.8;

        Vector p2[] = {Vector(x0, y0, z), Vector(x1, y1, z), Vector(x2, y2, z), Vector(x3, y3, z),
                       Vector(-x3, y3, z), Vector(-x2, y2, z), Vector(-x1, y1, z), Vector(x0, y0, z)};

        body.addNode(BezierCurve(p2, 8));

        //----nyak----------------------------
        x0 = 0;
        y0 = -0.68 - 0.4;
        x1 = 0.21 - 0.5;
        y1 = -0.58 - 0.4;
        x2 = 0.26 - 1;
        y2 = -0.35 + 1.4;
        x3 = 0.11 - 0.5;
        y3 = -0.17 + 1.4;
        z = 1;

        Vector p3[] = {Vector(x0, y0, z), Vector(x1, y1, z), Vector(x2, y2, z), Vector(x3, y3, z),
                       Vector(-x3, y3, z), Vector(-x2, y2, z), Vector(-x1, y1, z), Vector(x0, y0, z)};

        body.addNode(BezierCurve(p3, 8));

        //----++----------------------------
        x0 = 0;
        y0 = -0.68 - 1.4 + 2.2;
        x1 = 0.21 + 1.5;
        y1 = -0.58 - 1.4 + 2.2;
        x2 = 0.26 + 2.5;
        y2 = -0.35 - 2.7 + 2.2;
        x3 = 0.11 + 1.5;
        y3 = -0.17 - 2.7 + 2.2;
        z = 1.0;

        Vector p4[] = {Vector(x0, y0, z), Vector(x1, y1, z), Vector(x2, y2, z), Vector(x3, y3, z),
                       Vector(-x3, y3, z), Vector(-x2, y2, z), Vector(-x1, y1, z), Vector(x0, y0, z)};

        body.addNode(BezierCurve(p4, 8));

        //----has----------------------------
        x0 = 0;
        y0 = -1.9 - 0.5 + 1;
        x1 = 0.59 + 0.5;
        y1 = -1.61 - 0.5 + 1;
        x2 = 0.75 + 1;
        y2 = -0.96 + 2.6 + 1;
        x3 = 0.33 + 0.5;
        y3 = -0.44 + 2.6 + 1;
        z = 1.85;

        Vector p5[] = {Vector(x0, y0, z), Vector(x1, y1, z), Vector(x2, y2, z), Vector(x3, y3, z),
                       Vector(-x3, y3, z), Vector(-x2, y2, z), Vector(-x1, y1, z), Vector(x0, y0, z)};

        body.addNode(BezierCurve(p5, 8));

        //----has-vege----------------------------
        x0 = 0;
        y0 = -1.99 - 0.4 + 1;
        x1 = 0.23 - 1;
        y1 = -1.88 - 0.4 + 1;
        x2 = 0.29 - 2;
        y2 = -1.62 + 0.4 + 1;
        x3 = 0.12 - 1;
        y3 = -1.44 + 0.4 + 1;
        z = 2.94 - 1.5;

        Vector p6[] = {Vector(x0, y0, z), Vector(x1, y1, z), Vector(x2, y2, z), Vector(x3, y3, z),
                       Vector(-x3, y3, z), Vector(-x2, y2, z), Vector(-x1, y1, z), Vector(x0, y0, z)};

        body.addNode(BezierCurve(p6, 8));

        //----farok----------------------------
        x0 = 0;
        y0 = -1.94 + 0.0 + 1;
        x1 = 0.69 + 0.5;
        y1 = -1.92 + 0.0 + 1;
        x2 = 0.86 + 1;
        y2 = -1.88 + 0.0 + 1;
        x3 = 0.39 + 0.5;
        y3 = -1.84 + 0.0 + 1;
        z = 4.3 - 1.5;

        Vector p7[] = {Vector(x0, y0, z), Vector(x1, y1, z), Vector(x2, y2, z), Vector(x3, y3, z),
                       Vector(-x3, y3, z), Vector(-x2, y2, z), Vector(-x1, y1, z), Vector(x0, y0, z)};

        body.addNode(BezierCurve(p7, 8));
        body.make(_type, _position, new Material(Color(0.0, 0.0, 0.0), Color(0.0, 0.0, 0.0), Color(6, 6, 6), 50));
        //--------------------------------------------------------------------------------------------------------
        //-------FIN-----------------------------
        Vector points1[9] = {Vector(-0.12, 0.01, -0.12), Vector(-0.26, 0.05, -0.11), Vector(-0.32, 0.09, -0.1),
                             Vector(-0.25, 0.11, -0.09),
                             Vector(-0.11, 0.1, -0.09), Vector(0.03, 0.07, -0.09), Vector(0.08, 0.02, -0.1),
                             Vector(0.02, 0, -0.11), Vector(-0.12, 0.01, -0.12)};
        fin.addNode(BezierCurve(points1, 9));

        Vector points2[9] = {Vector(-0.13, -0.04, 0.09), Vector(-0.28, 0, 0.09), Vector(-0.33, 0.03, 0.1),
                             Vector(-0.27, 0.05, 0.11),
                             Vector(-0.12, 0.04, 0.11), Vector(0.02, 0.01, 0.11), Vector(0.07, -0.02, 0.1),
                             Vector(0.01, -0.05, 0.09), Vector(-0.13, -0.04, 0.09)};
        fin.addNode(BezierCurve(points2, 9));

        Vector points3[9] = {Vector(-0.23, -0.12, 0.23), Vector(-0.38, -0.08, 0.23), Vector(-0.43, -0.04, 0.24),
                             Vector(-0.36, -0.02, 0.25),
                             Vector(-0.22, -0.03, 0.25), Vector(-0.079, -0.07, 0.25), Vector(-0.02, -0.11, 0.24),
                             Vector(-0.08, -0.13, 0.23), Vector(-0.23, -0.12, 0.23)};
        fin.addNode(BezierCurve(points3, 9));

        Vector points4[9] = {Vector(-0.45, -0.2, 0.35), Vector(-0.6, -0.17, 0.36), Vector(-0.65, -0.13, 0.37),
                             Vector(-0.59, -0.1, 0.37),
                             Vector(-0.44, -0.11, 0.38), Vector(-0.29, -0.15, 0.38), Vector(-0.24, -0.19, 0.37),
                             Vector(-0.3, -0.21, 0.36), Vector(-0.45, -0.2, 0.35)};
        fin.addNode(BezierCurve(points4, 9));

        fin.make(_type, Vector(0, 0, 0), new Material(Color(0.0, 0.0, 0.0), Color(0.0, 0.0, 0.), Color(3, 3, 3), 30));
        //--------------------------------------------------------------------------------------------------------------------------------

        Color texture[5] = {Color(1, 1, 1), Color(1, 1, 1), Color(1, 1, 1), Color(.5, .2, .1), Color(0, 0, 0)};
        eyes[0] = 0.1;
        eyes[0].make(EYE, Vector(-0.2, 0.4, -0.7), NULL, new Texture(1, 5, texture));

        eyes[1] = 0.1;
        eyes[1].make(EYE, Vector(0.2, 0.4, -0.7), NULL, new Texture(1, 5, texture));

        body.moveHead(0, 0);
    }

    void rotateHead(float angle, Vector axis) {
        body.moveHead(angle * axis[1], angle * axis[0]);
    }

    void moveSeal(Vector delta) {
        sealPos = sealPos + delta;
        body.transalte(sealPos);

    }

    void getAngle(float &angle, Vector &axis, Vector eye, Vector ball) {

        Vector A = eye;
        Vector B = ball;

        A.Normalize();
        B.Normalize();

        angle = acos(A * B);
        axis = A % B;
    }

    void draw(float time) {
        body.begin();

#ifdef _DEBUG
        glPushMatrix();
        tail.draw(time);
        head.draw(time);
        glPopMatrix();
#endif

        body.draw(time);


        //--------------fin-------------------------------
        fin.transalte(Vector(0.1, 0.1, 0.8));
        fin.begin();
        glRotatef(120, 0, 1, 0);
        glRotatef(40, 1, 0, 0);
        glScalef(1.5, 1.5, 1.5);
        fin.draw(time);
        fin.end();

        fin.transalte(Vector(-0.4, 0.1, 0.5 + 0.8));
        fin.begin();
        glRotatef(120 - 50, 0, 1, 0);
        glRotatef(-40, 1, 0, 0);
        glScalef(1.5, 1.5, -1.5);
        fin.draw(time);
        fin.end();
        //--------------------------------------------------

        float angle = 0;
        //eye0
        glPushMatrix();
        glTranslatef(head.center.x, head.center.y, head.center.z);
        glRotatef(-head.angleY * 180 / PI, 1, 0, 0);
        glRotatef(head.angleX * 180 / PI, 0, 1, 0);
        glTranslatef(eyes[0].getPosition().x, eyes[0].getPosition().y, eyes[0].getPosition().z);

        getAngle(angle, axis, headPos, ballPos);
        glRotatef(-angle * 180 / PI, axis.x, axis.y, axis.z);
        glRotatef(25, 1, 0, 0);
        //glRotatef(-head.angleX*180/PI,0,1,0);
        glRotatef(head.angleY * 180 / PI, 1, 0, 0);
        eyes[0].draw(time);
        glPopMatrix();

        //eye1
        glPushMatrix();
        glTranslatef(head.center.x, head.center.y, head.center.z);
        glRotatef(-head.angleY * 180 / PI, 1, 0, 0);
        glRotatef(head.angleX * 180 / PI, 0, 1, 0);
        glTranslatef(eyes[1].getPosition().x, eyes[1].getPosition().y, eyes[1].getPosition().z);

        getAngle(angle, axis, headPos, ballPos);
        glRotatef(-angle * 180 / PI, axis.x, axis.y, axis.z);
        glRotatef(25, 1, 0, 0);
        glRotatef(head.angleY * 180 / PI, 1, 0, 0);
        eyes[1].draw(time);
        glPopMatrix();
        glPopMatrix();

        headPos = headPosAvg + body.getPosition();
        eye0 = eyes[0].getPosition() + body.getPosition() + head.center;
        eye1 = eyes[1].getPosition() + body.getPosition() + head.center;

    }
};

class Camera {
private:
    Vector eye, lookat, up;
    GLdouble fov, asp, fp, bp;
public:
    Camera(Vector _eye, Vector _lookat, Vector _up, GLdouble _fov = 60, GLdouble _asp = 1, GLdouble _fp = 1,
           GLdouble _bp = 100) {
        eye = _eye;
        lookat = _lookat;
        up = _up;
        fov = _fov;
        asp = _asp;
        fp = _fp;
        bp = _bp;
    }

    void apply() {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(fov, asp, fp, bp);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(eye.x, eye.y, eye.z, lookat.x, lookat.y, lookat.z, up.x, up.y, up.z);
    }

    void forward() {
        Vector delta = (lookat - eye) * 0.1;
        lookat = lookat + delta;
        eye = eye + delta;
    }

    void back() {
        Vector delta = (lookat - eye) * 0.1;
        lookat = lookat - delta;
        eye = eye - delta;
    }
};

class Light {
    Vector start;
    Color lin;
    Sphere light;
public:
    Light(Vector _position, Color _lin) {
        start = _position;
        lin = _lin;

        light = 0.1;
        light.make(SPHERE, _position, new Material(_lin, _lin, Color(0, 0, 0), 0));
    }

    Vector getStartPosition() {
        return start;
    }

    Sphere *getLight() {
        return &light;
    }

    void apply(int id) {
        glLightfv(GL_LIGHT0 + id, GL_AMBIENT, &lin.r);
        glLightfv(GL_LIGHT0 + id, GL_DIFFUSE, &lin.r);
        glLightfv(GL_LIGHT0 + id, GL_SPECULAR, &lin.r);
        Vector pos = light.getPosition();
        glLightfv(GL_LIGHT0 + id, GL_POSITION, (GLfloat *) &pos);
        glEnable(GL_LIGHT0 + id);
    }

    void draw(float time) {
        light.begin();
        light.draw(time);
        light.end();
    }
};

bool collidingCircles(Vector head, float radius1, Vector ball, float radius2) {
    Vector v = head - ball;

    float szumma = radius1 + radius2;
    float dist = v.x * v.x + v.y * v.y + v.z * v.z;

    if (dist <= szumma * szumma)
        return true;
    else
        return false;
}

bool collidingPlane(Vector pos, Plane *plane) {

    if (plane->getPoint(0).x < pos.x && plane->getPoint(2).x > pos.x &&
        plane->getPoint(0).z < pos.z && plane->getPoint(2).z > pos.z &&
        plane->getPoint(0).y > pos.y) {
        return true;
    }

    return false;
}

class Scene {
private:
    Object *objects[100];
    int numObjects;
    Light *lights[10];
    int numLights;
    Camera *camera;
public:
    Scene() {
        numObjects = 0;
        numLights = 0;
    }

    Camera *getCamera() {
        return camera;
    }

    void makeDrawable() {
        camera = new Camera(Vector(2, -0.6, -1), Vector(0, -0.7, 3), Vector(0, 1, 0));

        Color texture_room[64 * 64];
        for (int i = 0; i < 64; i++) {
            for (int j = 0; j < 64; j++) {
                if (i % 6 == 0 || j % 6 == 0)
                    texture_room[i * 64 + j] = Color(0.1, 0.1, 0.1);
                else
                    texture_room[i * 64 + j] = Color(1.0, 1.0, 1.0);
            }
        }

        Vector mesh[] = {Vector(-10, -4, 0),
                         Vector(-10, -4, 20),
                         Vector(10, -4, 20),
                         Vector(10, -4, 0)
        };

        Plane *floor = new Plane(mesh);
        floor->make(PLANE, Vector(0.0, 0.0, 0.0),
                    new Material(Color(0.0, 0.0, 0.0), Color(0.8, 0.8, 0.8), Color(2, 2, 2), 25),
                    new Texture(64, 64, texture_room));
        objects[numObjects++] = floor;

        Plane *lRoom = new Plane(mesh);
        lRoom->make(PLANE, Vector(0, 0, 0),
                    new Material(Color(0.0, 0.0, 0.0), Color(0.8, 0.8, 0.8), Color(2, 2, 2), 25),
                    new Texture(64, 64, texture_room));
        lRoom->rotate(90, Vector(0, 0, 1));
        lRoom->transalte(Vector(6, 2, 0));
        lRoom->scalef(Vector(1, .5, 1));
        objects[numObjects++] = lRoom;

        Plane *rRoom = new Plane(mesh);
        rRoom->make(PLANE, Vector(0, 0, 0),
                    new Material(Color(0.0, 0.0, 0.0), Color(0.8, 0.8, 0.8), Color(2, 2, 2), 25),
                    new Texture(64, 64, texture_room));
        rRoom->rotate(90, Vector(0, 0, 1));
        rRoom->transalte(Vector(6, -2, 0));
        rRoom->scalef(Vector(-1, -.5, 1));
        objects[numObjects++] = rRoom;

        Plane *bRoom = new Plane(mesh);
        bRoom->make(PLANE, Vector(0, 0, 0),
                    new Material(Color(0.0, 0.0, 0.0), Color(0.8, 0.8, 0.8), Color(2, 2, 2), 25),
                    new Texture(64, 64, texture_room));
        bRoom->rotate(90, Vector(1, 0, 0));
        bRoom->transalte(Vector(0, 12, -15));
        bRoom->scalef(Vector(1, .5, -1));
        objects[numObjects++] = bRoom;

        Color texture_ball[18];

        for (int i = 0; i < 18; i++) {
            if (i % 2 == 0)
                texture_ball[i] = Color(1, 1, 0);
            else
                texture_ball[i] = Color(1, 0, 0);
        }

        Ball *ball = new Ball(ballRadius);
        ballPos = Vector(-1.0, 2.0, 3.8);
        ball->make(BALL, ballPos, new Material(Color(0.1, 0.1, 0.1), Color(0.7, 0.7, 0.7), Color(2, 2, 2), 10),
                   new Texture(18, 1, texture_ball));
        objects[numObjects++] = ball;

#ifdef LOT_OF_BALL
        for (int i = 0; i < 20; i++)
        {
            float x = randFloat(-2.0,3.0);
            float y = randFloat(2.0,3.6);
            float z = randFloat(1.5,6.0);

            Ball* ball = new Ball(ballRadius);
            ballPos = Vector(x,y,z);
            ball->make(BALL,ballPos,new Material(Color(0.1,0.1,0.1), Color(0.7,0.7,0.7),Color(2,2,2),10), new Texture(18 ,1 , texture_ball));

            objects[numObjects++] = ball;
        }
#endif

        sealPos = Vector(-1, -3.3, 3.8);
        objects[numObjects++] = new Seal(SEAL, sealPos);

        lights[numLights++] = new Light(Vector(3, -2.5, -6), Color(1.0, 1.0, 1.0));
        lights[numLights++] = new Light(Vector(-2, -2.8, 7), Color(.6, .6, .6));
    }

    void draw(float time) {
        camera->apply();

        for (int i = 0; i < numLights; i++) {
            Vector pos = lights[i]->getStartPosition();
            Vector newPos(pos.x * cosf(0.72f * time), pos.y/* * cosf(time)*/, pos.z * sinf(time * 0.8f));
            lights[i]->getLight()->transalte(newPos);
            lights[i]->apply(i);
            lights[i]->draw(time);
        }

        for (int i = 0; i < numObjects; i++) {
            objects[i]->begin();
            objects[i]->draw(time);
            objects[i]->end();
        }

#ifdef _DEBUG
        glPolygonMode(GL_FRONT, GL_LINE);
        glPolygonMode(GL_BACK, GL_LINE);

        glPushMatrix();
        glTranslatef(headPos.x,headPos.y,headPos.z);
        glutSolidSphere(headRadius,15,15);
        glPopMatrix();

        glPolygonMode(GL_FRONT, GL_FILL);
        glPolygonMode(GL_BACK, GL_FILL);
#endif
    }

    bool intersect(Vector pos, Vector &dir) {
        for (int i = 0; i < numObjects; i++) {
            if (objects[i]->getType() == PLANE) {
                if (collidingPlane(pos, (Plane *) objects[i])) {
                    Vector A = pos;
                    Vector B = Vector(pos.x, -4 + 0.2, pos.z);
                    Vector C = A - B;
                    C = C.Normalize();
                    dir = C * -1;
                    return true;
                }
            }
        }

        if (collidingCircles(headPos, headRadius, pos, ballRadius)) {
            Vector A = pos;
            Vector B = headPos;
            Vector C = A - B;
            C = C.Normalize();
            dir = C;
            return true;
        }
        return false;
    }

    void update(float time) {
        for (int i = 0; i < numObjects; i++) {
            if (objects[i]->getType() == BALL) {
                //x(t) = x0 + vx*t + 0.5*ax*t^2

                Vector pos = objects[i]->getPosition();
                Vector dir = Vector(0, 0, 0);

                if (intersect(pos, dir)) {
                    float force = g / 2 * 4;
                    //dir*force;
                    ((Ball *) objects[i])->acc = dir * force;//Vector(0,g/2*3,0);
                }

                Vector a = ((Ball *) objects[i])->acc;
                pos.x += a.x * time;
                pos.y += -g / 2 + a.y;
                pos.z += a.z * time;

                ((Ball *) objects[i])->force(0.98);

                objects[i]->transalte(pos);
                ballPos = objects[i]->getPosition();
            }
        }
    }

    void keyDown(unsigned char key) {
        float angle = 10 * PI / 180;

        if (key == 's') {
            for (int i = 0; i < numObjects; i++) {
                if (objects[i]->getType() == SEAL) {
                    ((Seal *) objects[i])->rotateHead(angle, Vector(0, 1, 0));
                }
            }
        } else if (key == 'd') {
            for (int i = 0; i < numObjects; i++) {
                if (objects[i]->getType() == SEAL) {
                    ((Seal *) objects[i])->rotateHead(-angle, Vector(0, 1, 0));
                }
            }
        } else if (key == 'x') {
            for (int i = 0; i < numObjects; i++) {
                if (objects[i]->getType() == SEAL) {
                    ((Seal *) objects[i])->rotateHead(angle, Vector(1, 0, 0));
                }
            }
        } else if (key == 'e') {
            for (int i = 0; i < numObjects; i++) {
                if (objects[i]->getType() == SEAL) {
                    ((Seal *) objects[i])->rotateHead(-angle, Vector(1, 0, 0));
                }
            }
        } else if (key == 'a') {
            for (int i = 0; i < numObjects; i++) {
                if (objects[i]->getType() == SEAL) {
                    ((Seal *) objects[i])->moveSeal(Vector(0.0, 0.0, -0.1));
                }
            }
        } else if (key == 'y') {
            for (int i = 0; i < numObjects; i++) {
                if (objects[i]->getType() == SEAL) {
                    ((Seal *) objects[i])->moveSeal(Vector(0.0, 0, 0.1));
                }
            }
        }
    }
} scene;

void onInitialization() {
    glViewport(0, 0, screenWidth, screenHeight);
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);

#ifdef _DEBUG
    qobj = gluNewQuadric();
#endif

    tail.make(Vector(0, -0.8, 2.5), 2.0, 0, -PI / 5 + PI, 0);
    head.make(Vector(0, 0.0, 0.0), 0.5, 0, 0, 0);
    head.center = Vector(0, 0.5, 1.0);
    scene.makeDrawable();
}

void onDisplay() {
    glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float time = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    scene.draw(time);

#ifdef _DEBUG
    glBegin(GL_LINES);
    glVertex3f(headPos.x,headPos.y,headPos.z);
    glVertex3f(ballPos .x,ballPos.y,ballPos.z);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(eye0.x,eye0.y,eye0.z);
    glVertex3f(ballPos .x,ballPos.y,ballPos.z);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(eye1.x,eye1.y,eye1.z);
    glVertex3f(ballPos .x,ballPos.y,ballPos.z);
    glEnd();
#endif

    glutSwapBuffers();
}

void onKeyboard(unsigned char key, int x, int y) {
    if (key == 'f') scene.getCamera()->forward();
    else if (key == 'b') scene.getCamera()->back();

    scene.keyDown(key);
}

void onMouse(int button, int state, int x, int y) {
}

void onIdle() {
    float time = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    scene.update(time);
    glutPostRedisplay();
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitWindowSize(600, 600);
    glutInitWindowPosition(100, 100);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);

    glutCreateWindow("Seal skeleton - lesson 04");

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    onInitialization();

    glutDisplayFunc(onDisplay);
    glutMouseFunc(onMouse);
    glutIdleFunc(onIdle);
    glutKeyboardFunc(onKeyboard);

    glutMainLoop();

    return 0;
}
