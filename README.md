# stm32f7-gpio-cpp
CPP template-based driver for STM32F7 family GPIOs

# Short example - setting LED according to button state (F746G-DISCO)
 
  GPIO_bit<GPIO_Mode::output, GPIO_Port::I, 1> led(GPIO_Driver::push_pull, GPIO_Speed::low, GPIO_PullUpDown::floating, 0); 
  GPIO_bit<GPIO_Mode::input, GPIO_Port::I, 11> btn(GPIO_PullUpDown::up);

  while (1)
  {
    led = btn;
  }

# Two main templates
- GPIO_bit for configuring and operating on single bit of given port
- GPIO_bits for configuring many bits of a given port (chosen by mask or as a list of pin numbers, with use of additional helper template)

# GPIO_bit

  GPIO_bit<GPIO_Mode::analog, GPIO_Port::A, 10> PA10_analog;              // creating an object
  
  GPIO_bit<GPIO_Mode::analog, GPIO_Port::A, 10>::init();                  // using class method / static method 

# GPIO_bits

  GPIO_bits<GPIO_Port::G, 0x000e, GPIO_Mode::alternate>::init(GPIO_Driver::push_pull, GPIO_Speed::low, GPIO_PullUpDown::floating, 6);   
  
  GPIO_bits<GPIO_Port::G, BitNumsToMask<1,2,3>, GPIO_Mode::alternate>::init(GPIO_Driver::push_pull, GPIO_Speed::low, GPIO_PullUpDown::floating, 6);   


# TODO
- unify both templates into one


