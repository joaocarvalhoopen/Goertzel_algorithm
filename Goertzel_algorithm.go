// Name: Goertzel_algorithm.go
//
// Author:  Joao Nuno Carvalho
// Email:   joaonunocarv@gmail.com
// Date:    2017.12.10
// License: MIT OpenSource License
//
// Description: This is a implementation in Go ( GoLang ) programming language
//              of the Goertzel algorithm. The algorithm permits to determine
// the real and imaginary component of a frequency of monitoring in a signal.
// It also permit’s to know the magnitude of the vector. In the case, that one
// want’s to monitor a small number of frequencies this algorithm is more
// efficient then the FFT (Fast Fourier Transform).
// It’s is also very important when it’s necessary to distribute the computing
// time by each sample moment of processing, like for example in a interrupt
// service routine, of each sample that is acquired by a ADC. Unlike the FFT that
// has to perform the calculations in one block. One other advantage
// is that the buffer doesn’t have to be a power of 2 like in the FFT.
// And last, the frequency to be monitoring can be any specific frequency and
// to be precise doesn’t have to be the middle of a bucket like the FFT.
// The code was a port from C to Go, with several modification to permit that
// several frequencies could be monitored at the same time. The original work
// only permitted one frequency at a time.
//
// The original code in C was made by Kevin Bank, and he has a great article
// explaining the algorithm. The Link for the article is:
//    https://www.embedded.com/design/configurable-systems/4024443/The-Goertzel-Algorithm
// The link for the original code in C is:
//    https://www.embedded.com/design/embedded/source-code/4209931/09banks-txt
//
package main

import
   ("fmt"
	"math"
)


//##########################
//## Start of Goertzel code.

type FrequencyBucket struct {
	TargetFrequency float32     // 941.0	//941 Hz

	coeff  float32
	Q1     float32
	Q2     float32
	sine   float32
	cosine float32

	RealPart         float32  // Real part result
	ImagPart         float32  // Imaginary part result
	MagnitureSquared float32  // Magnitude squared result.
}

type Goertzel struct {
	SampleRate int // example 8000.0 //8kHz
	BufferSize int // Can be any size but has to be at least the doble of the highest
	                 // frequency that we whant to detect.
	freqBucket []*FrequencyBucket // The slice of frequencies we want to detect and all it's internal machinery.
	}

// Call this once, to precompute the constants.
func InitGoertzel(sampleRate int, bufferSize int, frequenciesList []float32 ) (G Goertzel) {

	// Allocate the memory for the structure int the return object.
	//G := Goertzel{}
	G.SampleRate = sampleRate
	G.BufferSize = bufferSize

	// Creates a frequency bucket for each frequency.
	for _, freq := range frequenciesList{
		fB := FrequencyBucket{}
		fB.TargetFrequency = freq
		G.freqBucket = append(G.freqBucket, &fB)
	}

	fmt.Printf("For SAMPLING_RATE = %d", G.SampleRate);
	fmt.Printf(" Buffer size = %d\n", G.BufferSize);

	// Precompute the constantes for each frequency.
	for _, freqB := range G.freqBucket {
		var floatN float32 = float32(G.BufferSize);
		var k int = int(0.5 + ((floatN * freqB.TargetFrequency) / float32(G.SampleRate)))
		var omega float32 = (2.0 * math.Pi * float32(k)) / floatN;
		freqB.sine = float32(math.Sin(float64(omega)))
		freqB.cosine = float32(math.Cos(float64(omega)))
		freqB.coeff = 2.0 * freqB.cosine;
		fmt.Printf(" and FREQUENCY = %f,\n", freqB.TargetFrequency);
		fmt.Printf("k = %d and coeff = %f\n\n", k, freqB.coeff);
		G.ResetGoertzel();
	}
	return G
}

// Call this routine before every "block" (size=N) of samples.
func (G *Goertzel) ResetGoertzel(){
	// Reset's the internal state.
	for _, freqBucket:= range G.freqBucket {
		freqBucket.Q1 = 0
		freqBucket.Q2 = 0
	}
}

// Call this routine for every sample.
func (G *Goertzel) ProcessSample(sample float32) {
	for _, fB := range G.freqBucket {
		var Q0 float32 = fB.coeff * fB.Q1 - fB.Q2 + sample
		fB.Q2 = fB.Q1
		fB.Q1 = Q0
	}
}

// Basic Goertzel
// Call this routine after every block to get the complex result.
func (G *Goertzel) CalcRealImag()  {
	for _, fB := range G.freqBucket {
		fB.RealPart = (fB.Q1 - fB.Q2 * fB.cosine)
		fB.ImagPart = (fB.Q2 * fB.sine)
	}
}

// Optimized Goertzel
// Call this after every block to get the RELATIVE magnitude squared.
func (G *Goertzel) calcMagnitudeSquared() {
	for _, fB := range G.freqBucket {
		fB.MagnitureSquared = fB.Q1 * fB.Q1 + fB.Q2 * fB.Q2 - fB.Q1 * fB.Q2 * fB.coeff;
	}
}

//## End of Goertzel code.
//#######################

const BUFFER_SIZE int = 205  // The size of the buffer doesn't need to be a power of 2.
const SAMPLE_RATE int = 8000 // Samples per second

var testData [BUFFER_SIZE]float32 = [BUFFER_SIZE]float32{}

// Synthesize some test data at a given frequency.
func Generate(frequency float32) {
	var step float32 = frequency * ((2.0 * math.Pi) / float32(SAMPLE_RATE));

	// Generate the test data.
	for index:=0; index < BUFFER_SIZE; index++ {
		testData[index] = float32(100.0 * math.Sin(float64(float32(index) * step)) + 100.0);
	}
}

// Demo 1
func GenerateAndTest(frequency float32){
	var magnitudeSquared float32
	var magnitude        float32
	var real             float32
	var imag             float32

	fmt.Printf("For test frequency %f:\n", frequency)
	Generate(frequency)

	// Process the samples.
	for index:=0; index < BUFFER_SIZE; index++ {
		goert.ProcessSample(testData[index])
	}

	// Do the "basic Goertzel" processing.
	goert.CalcRealImag()
	real = goert.freqBucket[0].RealPart
	imag = goert.freqBucket[0].ImagPart
	fmt.Printf("real = %f imag = %f\n", real, imag)

	magnitudeSquared = real*real + imag*imag
	fmt.Printf("Relative magnitude squared = %f\n", magnitudeSquared)
	magnitude = float32(math.Sqrt(float64(magnitudeSquared)))
	fmt.Printf("Relative magnitude = %f\n", magnitude)

	// Do the "optimized Goertzel" processing.
	goert.calcMagnitudeSquared()
	magnitudeSquared = goert.freqBucket[0].MagnitureSquared
	fmt.Printf("Relative magnitude squared = %f\n", magnitudeSquared)
	magnitude = float32(math.Sqrt(float64(magnitudeSquared)))
	fmt.Printf("Relative magnitude = %f\n\n", magnitude)

	goert.ResetGoertzel()
}

// Demo 2
func GenerateAndTest2(frequency float32) {
	var magnitudeSquared float32
	var magnitude        float32
	var real             float32
	var imag             float32

	fmt.Printf("Freq=%7.1f   ", frequency);
	Generate(frequency);

	// Process the samples.
	for index:=0; index < BUFFER_SIZE; index++	{
		goert.ProcessSample(testData[index]);
	}

	// Do the "standard Goertzel" processing.
	goert.CalcRealImag();
	real = goert.freqBucket[0].RealPart
	imag = goert.freqBucket[0].ImagPart

	magnitudeSquared = real*real + imag*imag;
	fmt.Printf("rel mag^2=%16.5f   ", magnitudeSquared);
	magnitude = float32(math.Sqrt(float64(magnitudeSquared)));
	fmt.Printf("rel mag=%12.5f\n", magnitude);

	goert.ResetGoertzel();
}

// Demo 3
// In this one the Goertzel Algorithm is looking (monitoring) for 2 frequencies.
func GenerateAndTest3(frequency float32) {

	fmt.Printf("Freq=%7.1f   ", frequency);
	Generate(frequency);

	// Process the samples.
	for index:=0; index < BUFFER_SIZE; index++	{
		goert.ProcessSample(testData[index]);
	}

	// Do the "standard Goertzel" processing.
	goert.CalcRealImag();
	real_A := goert.freqBucket[0].RealPart
	imag_A := goert.freqBucket[0].ImagPart
	real_B := goert.freqBucket[1].RealPart
	imag_B := goert.freqBucket[1].ImagPart

	magnitudeSquared_A := real_A*real_A + imag_A*imag_A;
	fmt.Printf("rel_A mag^2=%16.5f   ", magnitudeSquared_A);
	magnitude_A := float32(math.Sqrt(float64(magnitudeSquared_A)));
	fmt.Printf("rel_A mag=%12.5f | ", magnitude_A);

	magnitudeSquared_B := real_B*real_B + imag_B*imag_B;
	fmt.Printf("rel_B mag^2=%16.5f   ", magnitudeSquared_B);
	magnitude_B := float32(math.Sqrt(float64(magnitudeSquared_B)));
	fmt.Printf("rel_B mag=%12.5f\n", magnitude_B);

	goert.ResetGoertzel();
}

var goert Goertzel

func main() {
	fmt.Printf("Goertzel_algorithm.\n")
	var freq float32

	// Initializes to monitor/analyse 1 simultaneos frequency.
	var targetFreq float32 = 941.0
  	frequncyList := []float32{targetFreq} //941 Hz
	goert = InitGoertzel(SAMPLE_RATE, BUFFER_SIZE, frequncyList )

	// Demo 1
	// With the monitor for 1 frequency, generates 3 frequencies, below,
	// on target and higher.
	GenerateAndTest(targetFreq - 250);
	GenerateAndTest(targetFreq);
	GenerateAndTest(targetFreq + 250);

	// Demo 2
	// With the monitor for 1 frequency, generates a range of frequencies from
	// -300 to + 300 of the center frequency.
	for freq = targetFreq - 300; freq <= targetFreq + 300; freq += 15{
		GenerateAndTest2(freq)
	}

	// Demo 3
	// Initializes to monitor/analyse 2 simultaneos frequencies.
	// You can initializes with any number of frequencies, but the buffer size
	// as always to be greater or equal to the double of the frequency.
	var targetFreq_A float32 = 950.0 // Hz
	var targetFreq_B float32 = 900.0 // Hz
	frequncyList_2 := []float32{targetFreq_A, targetFreq_B}
	goert = InitGoertzel(SAMPLE_RATE, BUFFER_SIZE, frequncyList_2 )

	for freq = targetFreq - 300; freq <= targetFreq + 300; freq += 15{
		GenerateAndTest3(freq)
	}
}

