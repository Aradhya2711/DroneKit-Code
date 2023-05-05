void sort(Employee* emp, int n)
{
    // Perform bubble sort on the employee names
    for(int i = 0; i < n - 1; i++) {
        for(int j = 0; j < n - i - 1; j++) {
            if(strcmp(emp[j].name, emp[j+1].name) > 0) {
                Employee temp = emp[j];
                emp[j] = emp[j+1];
                emp[j+1] = temp;
            }
        }
    }

    // Display the sorted employees
    for(int i = 0; i < n; i++) {
        printf("\nEmployee %d:\n", i+1);
        printf("Name: %s\n", emp[i].name);
        printf("Branch: %s\n", emp[i].branch);
        printf("Age: %d\n", emp[i].age);
        printf("Employee ID: %d\n", emp[i].Employee_ID);
        printf("Salary: %d\n", emp[i].salary);
    }
}
