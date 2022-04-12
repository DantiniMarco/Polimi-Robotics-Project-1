#include <iostream>

using namespace std;

/*
  Front left wheel = 1
  Front right wheel = 2
  Rear right wheel = 3 
  Rear left wheel = 4 
*/

/*
 * omega: velocità angolare del robot
 * vbx: velocità lineare del robot lungo x
 * vby: velocità lineare del robot lungo y
 * w1,...,w4: le velocità angolari delle 4 ruote (guardare lo schema di riferimento nelle slides
 * di Cudrano per il progetto -> le ruote diagonalmente opposte sono (1,3) e (2,4))
 * l,w e r sono i soliti parametri che conosciamo
 *
 *
 * L'angolo di orientazione di robot è arctan(vby/vbx)
 * La velocità lineare v del robot è sqrt(vbx^2 + vby^2)
 */
void calculateOmega() { 
    double l = 0.2, w = 0.169, r = 0.07,omega,vbx,vby;
    double w1, w2, w3, w4; // [rad/min]
    /*cout << "Insert parameters:" << endl;
    cout << "omega: " << endl;
    cin >> omega;
    cout << "vbx: " << endl;
    cin >> vbx;
    cout << "vby: " << endl;
    cin >> vby;*/
    w1 = (-(l+w)/r) * omega + vbx/r - vby/r;
    w2 = ((l+w)/r) * omega + vbx/r + vby/r;
    w3 = ((l+w)/r) * omega + vbx/r - vby/r;
    w4 = (-(l+w)/r) * omega + vbx/r + vby/r;
    /*cout << "w1: " << u1 << endl;
    cout << "w2: " << u2 << endl;
    cout << "w3: " << u3 << endl;
    cout << "w4: " << u4 << endl;*/
}


/*
 * La formula per il calcolo di vbx,vby e omega è riadattata alle nostre convenzioni consierando
 * che in quel PDF trovato da Simone scambiava la ruota 3 con la ruota 4
 */
void calculateVelocities() {
    double l = 0.2, w = 0.169, r = 0.07,omega,vbx,vby;
    double w1, w2, w3, w4;
    /*cout << "Insert parameters:" << endl;
    cout << "u1: " << endl;
    cin >> w1;
    cout << "u2: " << endl;
    cin >> w2;
    cout << "u3: " << endl;
    cin >> w3;
    cout << "u4: " << endl;
    cin >> w4;*/
    vbx = (w1 + w2 + w3 + w4) * (r/4.0) * 60.0; 
    vby = (-w1 + w2 - w3 + w4) * (r/4.0) * 60.0;
    omega = (-w1 + w2 + w3 - w4) * (r/4.0 / (l+w)) * 60.0;
    /*cout << "omega: " << omega << endl;
    cout << "vbx: " << vbx << endl;
    cout << "vby: " << vby << endl;*/

}

int main() {
    return 0;
}
