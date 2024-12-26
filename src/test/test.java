package test;

public class test {
	public static long delta = 15;

	public static void main(String[] args) {
		delta = Long.highestOneBit(delta);
		System.out.println(delta);

	}

}
