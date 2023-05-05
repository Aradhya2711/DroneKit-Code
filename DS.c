#include<stdio.h>
#include<stdlib.h>
#include<string.h>

typedef struct {
    char name[20];
    char branch[20];
    int age;
    int Employee_ID;
    int salary;
} Employee;

void read(Employee* emp, int n);
void display(Employee* emp, int n);
void search(Employee* emp, int n);
void more_than(Employee* emp, int n);
void sort(Employee* emp, int n);

int main(){
    
    int n, e;
    Employee* emp;
    printf("Enter number of employees:");
    scanf("%d",&n);

    
    emp = (Employee*) malloc(n * sizeof(Employee));

    printf("Enter details of employees:\n");
    read(emp, n);

    printf("Here are the details:\n");
    display(emp, n);

    printf("Search employee ID:");
    scanf("%d",&e);
    search(emp, n);

    more_than(emp, n);

    printf("Sorted in alphabetical order:\n");
    sort(emp, n);

    
    free(emp);
    
    return 0;
}

void read(Employee* emp, int n)
{
    for(int i = 0; i < n; i++) {
        printf("\nEmployee %d:\n", i+1);
        printf("Name: ");
        scanf("%s", emp[i].name);
        printf("Branch: ");
        scanf("%s", emp[i].branch);
        printf("Age: ");
        scanf("%d", &emp[i].age);
        printf("Employee ID: ");
        scanf("%d", &emp[i].Employee_ID);
        printf("Salary: ");
        scanf("%d", &emp[i].salary);
    }
}

void display(Employee* emp, int n)
{
    for(int i = 0; i < n; i++) {
        printf("\nEmployee %d:\n", i+1);
        printf("Name: %s\n", emp[i].name);
        printf("Branch: %s\n", emp[i].branch);
        printf("Age: %d\n", emp[i].age);
        printf("Employee ID: %d\n", emp[i].Employee_ID);
        printf("Salary: %d\n", emp[i].salary);
    }
}

void search(Employee* emp, int n)
{
    int found = 0;
    for(int i = 0; i < n; i++) {
        if(emp[i].Employee_ID == n) {
            printf("\nEmployee %d:\n", i+1);
            printf("Name: %s\n", emp[i].name);
            printf("Branch: %s\n", emp[i].branch);
            printf("Age: %d\n", emp[i].age);
            printf("Employee ID: %d\n", emp[i].Employee_ID);
            printf("Salary: %d\n", emp[i].salary);
            found = 1;
            break;
        }
    }
    if(!found) {
        printf("Employee not found!\n");
    }
}

void more_than(Employee* emp, int n)
{
    printf("Details of employees with salary more than 50000:\n");
    for(int i = 0; i < n; i++) {
        if(emp[i].salary > 50000) {
            printf("\nEmployee %d:\n", i+1);
            printf("Name: %s\n", emp[i].name);
            printf("Branch: %s\n", emp[i].branch);
            printf("Age: %d\n", emp[i].age);
            printf("Employee ID: %d\n", emp[i].Employee_ID);
            printf("Salary: %d\n", emp[i].salary);
        }
    }
}
void sort(Employee* emp, int n)
{
    
    for(int i = 0; i < n - 1; i++) {
        for(int j = 0; j < n - i - 1; j++) {
            if(strcmp(emp[j].name, emp[j+1].name) > 0) {
                Employee temp = emp[j];
                emp[j] = emp[j+1];
                emp[j+1] = temp;
            }
        }
    }

    
    for(int i = 0; i < n; i++) {
        printf("\nEmployee %d:\n", i+1);
        printf("Name: %s\n", emp[i].name);
        printf("Branch: %s\n", emp[i].branch);
        printf("Age: %d\n", emp[i].age);
        printf("Employee ID: %d\n", emp[i].Employee_ID);
        printf("Salary: %d\n", emp[i].salary);
    }
}
