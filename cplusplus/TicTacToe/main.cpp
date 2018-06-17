#include "mainFunctions.cpp"


int main() {
  string player1, player2;
  int input;
  int row, column;
  int gameOver;

  player= getUserName();
  player1 = player.player1;
  player2 = player.player2;
  promptUserName();



  for (int i = 0; i < SIZE*SIZE/2; ++i) {

    // Player 1 move
    cout<<player1<<", it's your turn"<<endl;
    cin>>input;
    row = input/10;
    column = input - row*10;
    game.setGameSpace(row, column, 'x');
    gameOver = check();
    if (gameOver != 0) {
      declare(player1, gameOver);
      break;
    }

    game.printInfo();

    // Player 2 move
    cout<<player2<<", it's your turn"<<endl;
    cin>>input;
    row = input/10;
    column = input - row*10;
    game.setGameSpace(row, column, 'o');
    gameOver = check();
    if (gameOver != 0) {
      declare(player2, gameOver);
      break;
    }

    game.printInfo();
  }

  // Check if tied
    cout<<"tied."<<endl;

  return 0;
}