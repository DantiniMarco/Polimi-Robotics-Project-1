#include <iostream>

using namespace std;

/*
 * omega: velocità angolare del robot
 * vbx: velocità lineare del robot lungo x
 * vby: velocità lineare del robot lungo y
 * u1,...,u4: le velocità angolari delle 4 ruote (guardare lo schema di riferimento nelle slides
 * di Cudrano per il progetto -> le ruote diagonalmente opposte sono (1,3) e (2,4))
 * l,w e r sono i soliti parametri che conosciamo
 *
 *
 * L'angolo di orientazione di robot è arctan(vby/vbx)
 * La velocità lineare v del robot è sqrt(vbx^2 + vby^2)
 */
void calculateU() {
    double l = 0.2, w = 0.169, r = 0.07,omega,vbx,vby;
    double u1, u2, u3, u4;
    cout << "Insert parameters:" << endl;
    cout << "omega: " << endl;
    cin >> omega;
    cout << "vbx: " << endl;
    cin >> vbx;
    cout << "vby: " << endl;
    cin >> vby;
    u1 = (-(l+w)/r)*omega + vbx/r - vby/r;
    u2 = ((l+w)/r)*omega + vbx/r + vby/r;
    u3 = ((l+w)/r)*omega + vbx/r - vby/r;
    u4 = (-(l+w)/r)*omega + vbx/r + vby/r;
    cout << "u1: " << u1 << endl;
    cout << "u2: " << u2 << endl;
    cout << "u3: " << u3 << endl;
    cout << "u4: " << u4 << endl;
}


/*
 * La formula per il calcolo di vbx,vby e omega è riadattata alle nostre convenzioni consierando
 * che in quel PDF trovato da Simone scambiava la ruota 3 con la ruota 4
 */
void calculateV() {
    double l = 0.2, w = 0.169, r = 0.07,omega,vbx,vby;
    double u1, u2, u3, u4;
    cout << "Insert parameters:" << endl;
    cout << "u1: " << endl;
    cin >> u1;
    cout << "u2: " << endl;
    cin >> u2;
    cout << "u3: " << endl;
    cin >> u3;
    cout << "u4: " << endl;
    cin >> u4;
    vbx = (u1+u2+u3+u4)*(r/4);
    vby = (-u1+u2+u4-u3)*(r/4);
    omega = (-u1+u2-u4+u3)*(r/4*(l+w));
    cout << "omega: " << omega << endl;
    cout << "vbx: " << vbx << endl;
    cout << "vby: " << vby << endl;

}

/*
void calculateV() {
    double u1, u2, u3, u4;
    double omega, vbx, vby;
    bool flag[4] = {true, true, true, true};
    int choice;
    int firstFree = 0;
    double A[4][3] = {{-5.271428571, 14.28571429, -14.28571429},
                      {5.271428571, 14.28571429, 14.28571429},
                      {5.271428571, 14.28571429, -14.28571429},
                      {-5.271428571, 14.28571429, 14.28571429}};
    double B[3][3];
    double det;
    double cof[3][3];
    double temp;
    cout << "Which variable do you want to exclude? (1/2/3/4)" << endl;
    cin >> choice;
    switch(choice) {
        case 1:
            flag[0] = false;
            break;
        case 2:
            flag[1] = false;
            break;
        case 3:
            flag[2] = false;
            break;
        case 4:
            flag[3] = false;
            break;
    }
    cout << "Insert parameters:" << endl;
    if(flag[0]) {
        cout << "u1: " << endl;
        cin >> u1;
        for(int i = 0; i < 3; i++) {
            B[firstFree][i] = A[0][i];
        }
        firstFree++;
    }
    if(flag[1]) {
        cout << "u2: " << endl;
        cin >> u2;
        for(int i = 0; i < 3; i++) {
            B[firstFree][i] = A[1][i];
        }
        firstFree++;
    }
    if(flag[2]) {
        cout << "u3: " << endl;
        cin >> u3;
        for(int i = 0; i < 3; i++) {
            B[firstFree][i] = A[2][i];
        }
        firstFree++;
    }
    if(flag[3]) {
        cout << "u4: " << endl;
        cin >> u4;
        for(int i = 0; i < 3; i++) {
            B[firstFree][i] = A[3][i];
        }
        firstFree++;
    }
    det = B[0][0]*(B[1][1]*B[2][2] - B[1][2]*B[2][1]) - B[0][1]*(B[1][0]*B[2][2] - B[2][0]*B[1][2])
            + B[0][2]*(B[1][0]*B[2][1] - B[1][1]*B[2][0]);
    cof[0][0] = B[1][1]*B[2][2] - B[1][2]*B[2][1];
    cof[0][1] = -(B[1][0]*B[2][2] - B[2][0]*B[1][2]);
    cof[0][2] = B[1][0]*B[2][1] - B[1][1]*B[2][0];
    cof[1][0] = -(B[0][1]*B[2][2] - B[2][1]*B[0][2]);
    cof[1][1] = B[0][0]*B[2][2] - B[0][2]*B[2][0];
    cof[1][2] = -(B[0][0]*B[2][1] - B[0][1]*B[2][0]);
    cof[2][0] = B[0][1]*B[1][2] - B[0][2]*B[1][1];
    cof[2][1] = -(B[0][0]*B[1][2] - B[0][2]*B[1][0]);
    cof[2][2] = B[0][0]*B[1][1] - B[0][1]*B[1][0];
    temp = cof[0][1];
    cof[0][1] = cof[1][0];
    cof[1][0] = temp;
    temp = cof[0][2];
    cof[0][2] = cof[2][0];
    cof[2][0] = temp;
    temp = cof[2][1];
    cof[2][1] = cof[1][2];
    cof[1][2] = temp;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            cof[i][j] /= det;
        }
    }
    switch(choice) {
        case 1:
            omega = cof[0][0]*u2 + cof[0][1]*u3 + cof[0][2]*u4;
            vbx = cof[1][0]*u2 + cof[1][1]*u3 + cof[1][2]*u4;
            vby = cof[2][0]*u2 + cof[2][1]*u3 + cof[2][2]*u4;
            break;
        case 2:
            omega = cof[0][0]*u1 + cof[0][1]*u3 + cof[0][2]*u4;
            vbx = cof[1][0]*u1 + cof[1][1]*u3 + cof[1][2]*u4;
            vby = cof[2][0]*u1 + cof[2][1]*u3 + cof[2][2]*u4;
            break;
        case 3:
            omega = cof[0][0]*u1 + cof[0][1]*u2 + cof[0][2]*u4;
            vbx = cof[1][0]*u1 + cof[1][1]*u2 + cof[1][2]*u4;
            vby = cof[2][0]*u1 + cof[2][1]*u2 + cof[2][2]*u4;
            break;
        case 4:
            omega = cof[0][0]*u1 + cof[0][1]*u2 + cof[0][2]*u3;
            vbx = cof[1][0]*u1 + cof[1][1]*u2 + cof[1][2]*u3;
            vby = cof[2][0]*u1 + cof[2][1]*u2 + cof[2][2]*u3;
            break;
    }
    cout << "omega: " << omega << endl;
    cout << "vbx: " << vbx << endl;
    cout << "vby: " << vby << endl;
}
 */

int main() {
    char choice;
    cout << "What do you want to calculate? (v/u)" << endl;
    cin >> choice;
    if (choice == 'u') {
        calculateU();
    }
    else if (choice == 'v') {
        calculateV();
    }
    return 0;
}
