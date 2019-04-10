package orTools;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;

import com.google.ortools.constraintsolver.DecisionBuilder;
import com.google.ortools.constraintsolver.IntExpr;
import com.google.ortools.constraintsolver.IntVar;
import com.google.ortools.constraintsolver.Solver;

public class embeddedOrTools {

	
	   static {
	        System.loadLibrary("jniortools");
	    }

	public static void solve (int numberSensor,Writer w) throws IOException {
		 Solver solver = new Solver("Sensors");
		 
		 IntVar[] s = solver.makeIntVarArray(numberSensor, 0, 1, "s");
		
		 int sqrtS=(int) Math.sqrt(numberSensor);
		    IntVar[] colIntVar = new IntVar[sqrtS]; 
		    IntVar[] rowIntVar= new IntVar[sqrtS];
		    IntVar[] nSensor = new IntVar[5];
		    IntVar subSquareSum =solver.makeIntVar(0, 20);
			
		   //first Quad 
			 for(int i=0;i<sqrtS;i++) {
				 if(i>0&&i<sqrtS/2){
				 for(int j=0;j<sqrtS;j++) {
					 if(j>0&&j<sqrtS/2) {
				
					 subSquareSum = solver.makeSum(subSquareSum, s[getPosition(i,j,sqrtS)]).var();
					 }
				 }
				 }
				 solver.addConstraint(solver.makeGreaterOrEqual(subSquareSum,2));
			 }
			//second Quad 
			 for(int i=0;i<sqrtS;i++) {
				 if(i>(int)sqrtS/2&&i<sqrtS){
				 for(int j=0;j<sqrtS;j++) {
					 if(j>0&&j<sqrtS/2) {
				
					 subSquareSum = solver.makeSum(subSquareSum, s[getPosition(i,j,sqrtS)]).var();
					 }
				 }
				 }
				 solver.addConstraint(solver.makeGreaterOrEqual(subSquareSum, 2));
			 }
			 //third quad
			 for(int i=0;i<sqrtS;i++) {
				 if(i>0&&i<sqrtS/2){
				 for(int j=0;j<sqrtS;j++) {
					 if(j>(int)sqrtS/2&&j<sqrtS){
				
					 subSquareSum = solver.makeSum(subSquareSum, s[getPosition(i,j,sqrtS)]).var();
					 }
				 }
				 }
				 solver.addConstraint(solver.makeGreaterOrEqual(subSquareSum, 2));
			 }
			 //fourth quad
			 for(int i=0;i<sqrtS;i++) {
				 if(i>(int)sqrtS/2&&i<sqrtS){
				 for(int j=0;j<sqrtS;j++) {
					 if(j>(int)sqrtS/2&&j<sqrtS){
				
					 subSquareSum = solver.makeSum(subSquareSum, s[getPosition(i,j,sqrtS)]).var();
					 }
				 }
				 }
				 solver.addConstraint(solver.makeGreaterOrEqual(subSquareSum,2));
			 }
			 
		//COLSUM	 
		 for(int i=0;i<sqrtS;i++) {
			 colIntVar[i]= solver.makeIntVar(0, 0);
			 rowIntVar[i]= solver.makeIntVar(0, 0);
			 for(int j=0;j<sqrtS;j++) {
				 colIntVar[i]= solver.makeSum(colIntVar[i].var(),s[getPosition(i,j,sqrtS)].var()).var();
				 rowIntVar[i]= solver.makeSum(rowIntVar[i].var(),s[getPosition(j,i,sqrtS)].var()).var();
		
			 }
			 if((i==0)||(i==sqrtS-1)) {
				 solver.addConstraint(solver.makeGreaterOrEqual(colIntVar[i].var(), (int)sqrtS/4 ));
				 solver.addConstraint(solver.makeGreaterOrEqual(rowIntVar[i].var(), (int)sqrtS/4 ));
			  
			 }else {
			 solver.addConstraint(solver.makeGreaterOrEqual(colIntVar[i].var(), (int)sqrtS/4 ));
			 solver.addConstraint(solver.makeGreaterOrEqual(rowIntVar[i].var(), (int)sqrtS/4 ));
		 }
			 }

		 
		solver.makeMinimize(solver.makeSum(s).var(), 3);

		 IntVar fireS= solver.makeIntVar(numberSensor-1, numberSensor-1, "fs");
		 solver.addConstraint(solver.MakeElementEquality(s, fireS, 1));

		 

		 for(int col=0;col<sqrtS;col++) {//col
			 for(int row=0;row<sqrtS;row++) { //row
		
			if(row!=0) {//up
			 nSensor[0]=s[getPosition(row,col,sqrtS)-1].var();
			}else {
				 nSensor[0]=solver.makeIntConst(0);
			}
			if(row!=(sqrtS-1)) {//down
				nSensor[1]=s[getPosition(row,col,sqrtS)+1].var();
			}else {
				 nSensor[1]=solver.makeIntConst(0);
			}
			if(col!=0) {	//left
				nSensor[2]=s[getPosition(row,col,sqrtS)-1].var();
			}else {
				 nSensor[2]=solver.makeIntConst(0);
			}
			if(col!=sqrtS-1) {	//right
				nSensor[3]=s[getPosition(row,col,sqrtS)+1].var();
			}else {
				 nSensor[3]=solver.makeIntConst(0);
			}
			
			nSensor[4]=solver.makeIntConst(0);
			 solver.addConstraint(solver.makeSumGreaterOrEqual(nSensor, 1));
			 nSensor[4]= s[getPosition(row,col,sqrtS)];
			 solver.addConstraint(solver.makeSumGreaterOrEqual(nSensor, 2));
		 }
			 }
		
	
		
		  DecisionBuilder db = solver.makePhase(s,
                  solver.CHOOSE_FIRST_UNBOUND,
                 solver.ASSIGN_MIN_VALUE);
		  
		  	solver.newSearch(db);
		  	int solutionN=0;
		  	int tot=0;
		   // String text = "Hello world";
		  
	        
	     	
		  	w.write("start");
            
		  	int val=0;
		    while (solver.nextSolution()) {
		    	if(solutionN<1) {
		    		for(int i=0;i<numberSensor;i++) {
		    				tot=(int) (tot+s[i].value());
		    				val=(int)s[i].value();
		    	            w.write(val+";");
		    	            if(i%sqrtS>0) {
	                    System.out.print(s[i].value() + " ");
		    		 	   }else {
		    			System.out.println(s[i].value() +" ");
		    		}}
		    		w.write(("end;\n"));
		    		w.flush();
		    		solutionN++;
	                System.out.println("end"+ tot);
	                System.out.println("\n");
	                
//	                tot=0;
	            }else {
	            	 solver.endSearch();
	     		    w.close();
	            }
		    	solutionN++;
		    }

	    w.close();
	       solver.endSearch();
	        System.out.println("Solutions: " + solver.solutions());
	        System.out.println("Failures: " + solver.failures());
	        System.out.println("Branches: " + solver.branches());
	        System.out.println("Wall time: " + solver.wallTime() + "ms");
	}
	

		 
		 public static int getPosition(int i,int j,int n) {
			 
			 return (i*n)+j;
		 }
	 public static void main(String[] args) throws Exception {
		 
		  try {
	            //Whatever the file path is.
	            File statText = new File("output6.txt");
	            FileOutputStream is = new FileOutputStream(statText);
	            OutputStreamWriter osw = new OutputStreamWriter(is);    
	            Writer w = new BufferedWriter(osw);

	   		 embeddedOrTools.solve(121,w);
		  } catch (IOException e) {
	            System.err.println("Problem writing to the file statsTest.txt");
	        }
	 }
}
