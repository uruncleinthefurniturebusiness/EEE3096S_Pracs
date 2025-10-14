import java.io.*;
import java.lang.*;
import java.util.*;

public class Spelling{

    static int  row = 0;
    static int  col = 0;

   public Spelling(){}
   
   public static String check(String[][] grid, String[] trace, int length, int x, int y){
      int i = 0;
       
      if  (!grid[x][y].equals(trace[i])){
         return "";
      }
         
      else if (grid[x][y].equals(trace[i])){
         String[] arr = new String[length-1];
         
         if ((x+1 < row) && grid[x+1][y].equals(trace[i+1])){
            for (int j = 1; j<length; j++){
               arr[j-1] = trace[j]; 
            }
            return grid[x][y] + check(grid, arr, arr.length, x+1, y);
         }
         else if ((x-1 >=0) && grid[x-1][y].equals(trace[i+1])){
            for (int j = 1; j<length; j++){
               arr[j-1] = trace[j]; 
            }
            return grid[x][y] + check(grid, arr, arr.length, x-1, y);
         }
         else if ((y+1 < col) && grid[x][y+1].equals(trace[i+1]) ){
            for (int j = 1; j<length; j++){
               arr[j-1] = trace[j]; 
            }
            return grid[x][y] + check(grid, arr, arr.length, x, y+1);
         }
         else if ((y-1 >= 0) && grid[x][y-1].equals(trace[i+1])){
            for (int j = 1; j<length; j++){
               arr[j-1] = trace[j]; 
            }
            return grid[x][y] + check(grid, arr, arr.length, x, y-1);
         }
        
      }
      
      return "";
      
   }
      
   public static void main(String[] args){
      System.out.print("Enter the number of rows and columns:\n");
      Scanner input = new Scanner(System.in);
      
      String[] rc = new String[2]; 
      rc = input.nextLine().split(" ");
      
      int row = Integer.parseInt(rc[0]);
      int col = Integer.parseInt(rc[1]);
      
      String[][] grid = new String[row][col];
      
      System.out.print("Enter the rows of the grid:\n");     
      for (int i = 0; i < row; i++){
         String[] word = new String[col];
         word = input.nextLine().split("");
                  
         for (int j = 0; j < col; j++){
            grid[i][j] = word[j];
                            
         }  
      }
      
      System.out.print("Enter the starting position:\n");
      String[] start = new String[2]; 
      start = input.nextLine().split(" ");
      
      int r = Integer.parseInt(start[0]);
      int c = Integer.parseInt(start[1]);
               
      System.out.print("Enter the string to trace:\n");
      String trace = input.nextLine();
      int len = trace.length();
      String[] arr = new String[len];
      //trace = "";
      arr = trace.split("");
      
      
      String sol = check(grid, arr, len, r, c);
      
      
      System.out.print("Amount that could be traced: \'" + sol + "\'.");  
     
   
   }

}