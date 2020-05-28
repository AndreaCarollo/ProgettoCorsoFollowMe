#include <iostream>
#include <string>

using namespace std;

enum VendingMachineState
{
    SELECT,
    QUANTITY,
    CALCULATE,
    UPDATE
};

enum Brand
{
    COKE = 100,
    PEPSI = 200,
    SEVENUP = 300
};

struct Student
{
    string name;
    int points;
};

struct Soda
{
    Brand brandName;
    int inventory;
    int price;
};

int main()
{

    // Initial Vending Machine State is SELECT.
    VendingMachineState currentState = SELECT;

    //FIll up our vending mach with sodas
    Soda vendingMachine[3];
    //load coke
    vendingMachine[0].brandName = COKE;
    vendingMachine[0].inventory = 10;
    vendingMachine[0].price = 1000;
    //load pepsi
    vendingMachine[1].brandName = PEPSI;
    vendingMachine[1].inventory = 10;
    vendingMachine[1].price = 1000;
    //load sevenup
    vendingMachine[2].brandName = SEVENUP;
    vendingMachine[2].inventory = 10;
    vendingMachine[2].price = 1000;

    Soda selectedSoda;
    Student eric;
    eric.name = "Eric";
    eric.points = 1000000;

    switch (currentState)
    {
    case SELECT:
        //1. Display the current status for the vending machine and the student's  wallet
        cout << "<<< --- CURRENT STATUS --- >>>" << endl;
        cout << " [[[Vending Machine]]]" << endl;
        for (int i = 0; i < 3; i++)
        {
            cout << vendingMachine[i].brandName << " has " << vendingMachine[i].inventory << " drinks, and cost"
                 << vendingMachine[i].price << " each." << endl;
        }
        cout << "[[[Student]]]" << endl;
        cout << eric.name << " has " << eric.points << " points total" << endl;
        cout << "<<< --- END STATUS --- >>>" << endl
             << endl;
        //2. ask user what drik they want.
        cout << "Please select a drink from the following available options:" << endl;
        //3. Display the soda names and corresponfing ID values.
        cout << "COKE = 100\nPEPSI = 200\nSEVENUP = 300" << endl;
        //4. User inputs the drink.
        int selectedDrink;
        cin >> selectedDrink;
        //5. If valid drink selected, move to QUANTITY state; else, go bask to SELECT state.
        switch (selectedDrink)
        {
        case COKE:
            cout << "you have selected COKE" << endl;
            selectedSoda.brandName = COKE;
            currentState = QUANTITY;
            break;
        case PEPSI:
            cout << "you have selected PEPSI" << endl;
            selectedSoda.brandName = PEPSI;
            currentState = QUANTITY;
            break;
        case SEVENUP:
            cout << "you have selected SEVENUP" << endl;
            selectedSoda.brandName = SEVENUP;
            currentState = QUANTITY;
            break;
        default:
            cout << "Invalid drink selected!" << endl;
            currentState = SELECT;
            break;
        }
        break;
    case QUANTITY:

        cout << "How many drinks would you like?" << endl;
        int orderQuantity;
        cin >> orderQuantity;

        cout << "your order has been reveived. You placed an order " << orderQuantity << " soda(s)" << endl;
        switch (selectedSoda.brandName)
        {
        case COKE:
            break;
        case PEPSI:
            break;
        case SEVENUP:
            break;

        default:
            cout << "invalid amoun received" << endl;
            currentState = SELECT;
            break;
        }

        break;
    case CALCULATE:
        break;
    case UPDATE:
        break;
    default:
        cout << "Error! invalid state detected." << endl;
    }

    return 0;
}