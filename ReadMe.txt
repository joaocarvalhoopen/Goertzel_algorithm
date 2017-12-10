Name: Goertzel_algorithm.go

Author:  Joao Nuno Carvalho
Email:   joaonunocarv@gmail.com
Date:    2017.12.10
License: MIT OpenSource License

Description: This is a implementation in Go ( GoLang ) programming language of
             the Goertzel algorithm. The algorithm permits to determine the real
             and imaginary component of a frequency of monitoring in a signal.
It also permit’s to know the magnitude of the vector. In the case, that one want’s
to monitor a small number of frequencies this algorithm is more efficient then the
FFT (Fast Fourier Transform). It’s is also very important when it’s necessary to
distribute the computing time by each sample moment of processing, like for example
in a interrupt service routine, of each sample that is acquired by a ADC. Unlike the
FFT that has to perform the calculations in one block. One other advantage is that
the buffer doesn’t have to be a power of 2 like in the FFT. And last, the frequency
to be monitoring can be any specific frequency and to be precise doesn’t have to be
the middle of a bucket like the FFT.

The code was a port from C to Go, with several modification to permit that several
frequencies could be monitored at the same time. The original work only permitted
one frequency at a time.
The original code in C was made by Kevin Bank, and he has a great article explaining
the algorithm. The Link for the article is:
  https://www.embedded.com/design/configurable-systems/4024443/The-Goertzel-Algorithm
The link for the original code in C is:
  https://www.embedded.com/design/embedded/source-code/4209931/09banks-txt

