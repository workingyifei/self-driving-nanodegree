#include <iostream>
#include <iomanip>

using namespace std;


const int SIZE=4;

class Gameboard
{
    char gameSpace[SIZE][SIZE];

public:
    Gameboard(); //initialize the board with '-' in all 16 spaces
    void setGameSpace(int row,int column, char value); //x,y,or '-' in each game square
    char getGameSpace(int row,int column);
    int fourInRow(); //four 'x's in any row 'wins'
    int fourInColumn(); //four 'x' in any column 'wins'
    int fourInDiagonal(); //four 'x' in any diagonal 'wins'
    void printInfo(); //print the game board in a 4x4 matrix
};


Gameboard::Gameboard()
{
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            gameSpace[i][j] = '-';
}

void Gameboard::setGameSpace(int row, int column, char value)
{
    gameSpace[row][column] = value;
}

char Gameboard::getGameSpace(int row,int column)
{
    return gameSpace[row][column];
}

int Gameboard::fourInRow() {
    for(int i=0; i<SIZE; i++)  {
        int count = 0;
        for(int j=0; j<SIZE; j++) {
            if (gameSpace[i][j] == 'x')
                count++;
        }
        if (count == SIZE)
            return 1;
    }
}

int Gameboard::fourInColumn() {
    for(int i=0; i<SIZE; i++)   {
        int count = 0;
        for(int j=0; j<SIZE; j++) {
            if (gameSpace[j][i] == 'x')
                count++;
        }
        if (count == SIZE)
            return 1;
    }
}

int Gameboard::fourInDiagonal() {
    int count1 = 0;
    int count2 = 0;
    for (int i = 0; i < SIZE; i++) {
        if (gameSpace[i][i] == 'x')
            count1++;
    }

    for (int i = 0; i < SIZE; i++) {
        if (gameSpace[i][SIZE-1-i] == 'x')
            count2++;
    }

    if (count1 == SIZE || count2 == SIZE)
        return 1;
    }




void Gameboard::printInfo() {
     for(int i=0; i<SIZE; i++) {
         for (int j = 0; j < SIZE; j++)
             std::cout << gameSpace[i][j] << "\t";
         cout << endl;
     }
}