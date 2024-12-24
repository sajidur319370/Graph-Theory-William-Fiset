package travellingSalesmanProblem;

public class test {
	public static int fib(int n) {

		if (n == 0) {
			System.out.println("Returned!!!!!!!!!!!!!!!!!!!!");
			return 1;
		}
		for (int i = 0; i < n; i++) {
			System.out.println("iiiiiiii: " + i);
			System.out.println("Before N: " + n);
			fib(n - 1);
			System.out.println("After N: " + n);
		}
		return n;

	}

	public static void main(String[] args) {

		System.out.println(fib(4));

	}

}
