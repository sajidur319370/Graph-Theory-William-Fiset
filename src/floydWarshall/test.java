package floydWarshall;

import static java.lang.Double.NEGATIVE_INFINITY;
import static java.lang.Double.POSITIVE_INFINITY;

public class test {

	public static void main(String[] args) {
		int n = 7;
		double[][] m = new double[n][n];
		for (int i = 0; i < n; i++) {
			java.util.Arrays.fill(m[i], POSITIVE_INFINITY);
			m[i][i] = 0;
		}
		// Add some edge values.
		m[0][1] = 2;
		m[0][2] = 5;
		m[0][6] = 10;
		m[1][2] = 2;
		m[1][4] = 11;
		m[2][6] = 2;
		m[6][5] = 11;
		m[4][5] = 1;
		m[5][4] = -2;

		double[][] dp = new double[n][n];
		Integer[][] next = new Integer[n][n];
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				if (m[i][j] != POSITIVE_INFINITY) {
					next[i][j] = j;
				}
				dp[i][j] = m[i][j];
				System.out.print(dp[i][j] + "  ");
			}
			System.out.println("\n");
		}
		System.out.println("========================================\n");
		for (int k = 0; k < n; k++) {
			System.out.println("==============next:" + k + "\n");
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					if (dp[i][k] + dp[k][j] < dp[i][j]) {
						dp[i][j] = dp[i][k] + dp[k][j];
						next[i][j] = next[i][k];
					}
					System.out.print(dp[i][j] + "  ");
				}
				System.out.println("\n");
			}

		}
		for (int k = 0; k < n; k++) {
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					if (dp[i][k] != POSITIVE_INFINITY && dp[k][j] != POSITIVE_INFINITY && dp[k][k] < 0) {
						dp[i][j] = NEGATIVE_INFINITY;
						next[i][j] = -1;
					}
				}
			}
		}

	}

}
