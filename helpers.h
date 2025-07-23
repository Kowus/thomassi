float integrate(float (*func)(float), float a, float b, int n) {
  float h = (b - a) / n;                  // Step size
  float sum = 0.5 * (func(a) + func(b));  // Start with the endpoints
  for (int i = 1; i < n; i++) {
    sum += func(a + i * h);  // Add the middle terms
  }
  return sum * h;  // Multiply by the step size
}