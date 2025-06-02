## Examen-Final FPGA-implementacion

# 1) Alu de 8 bits

## Link del video de funcionamiento de la ALU 8 bits, con el codigo cargado a la basys 3: [FPGA - ALU 8 bits](https://youtu.be/IEkZB2Pj5U0)

Captura del routing de la Alu
![implmentacion alu 2](https://github.com/user-attachments/assets/b781bf8c-ecc7-46fa-b87f-edb65027d835)

Basicamente el codigo es casi el mismo que se utilizo en el segundo parcial. La diferencias clave estan en la implementacion de un sumador carry look-ahead, el cual funciona como submodulo y realiza las operacion de suma en todo momento. Sin embargo no significa que no pueda variar segun la combinacion de botones que se elija para las operaciones:

Las combinaciones posibles son de 6 bits y serian las siguientes:

- 000 = suma
- 100 = resta
- 010 = AND
- 110 = OR
- 001 = Right Shift
- 101 = Left Shift
- 011 = Arithmetic Right Shift
- 111 = No asignado / Sin operaciones

En el caso de la implementacion fisica para la eleccion de las operaciones, se debe tomar de la siguiente manera:

- op [0,0,0] - op -> operacion es un array que puede tener 3 valores.
  
Para seleccionar entre esas operaciones usamos:

- Boton left es igual a---op[0] -> posicion 0
- Boton up es igual a----op[1] -> posicion 1
- Boton right es igual a--op[2] -> posicion 2

```
`timescale 100ns / 1ps

`include "cla_8bits.sv"
 
module Alu_8bits(
    input  wire [7:0] A,  // entrada de A 8bit
    input  wire [7:0] B,  // entrada de B 8bit
    input  wire [2:0] op, // codigo para seleccionar operacion
    input  wire   sh_sel, // seleccion de que numero A o B se hace el shift
    output reg  [7:0] Y, // resultado de las operaciones
    output reg        C, // carry out
    output reg        Z, // si es cero
    output reg        N, // si es negativo
    output reg        V  // si hay overflow
    );
    // numero que elige para las operaciones de shift [Elegir A o B]
    wire [7:0] S = sh_sel ? A : B; // se shiftea A o  B ?

    // definicion de variables intermedias para las salidas
    // de la suma, resta y el carry correspondiente a cada una
    wire [7:0] sum_out, diff_out;
    wire sum_carry, diff_carry;

    // suma: Cin = 0, no hay carry in
    // se instacia el carry look ahead para la suma
    cla_8bits cla_sum (
        .A(A), // asignar a A CLA, el A del input de la ALU que viene de los switches.
        .B(B), // lo mismo para B, a CLA el input de la ALU.
        .Cin(1'b0), // carry igual a 0
        .Sum(sum_out), // asignar a la suma de CLA la suma de la ALU, es decir Sum
                       // "empuja" (da el valor) de la suma realizada a sum_out
        .Cout(sum_carry)
    );

    // resta: A + (~B + 1) Cin = 1, se usa carry in para calcular la resta
    // instacia para la resta con el carry look ahead adder
    wire [7:0] B_comp = ~B; // B complemento
    cla_8bits cla_diff (
        .A(A), // asignar a A CLA, el A del input de la ALU 
        .B(B_comp), // asignar a B de la CLA, el complemento de B de la entrada de ALU 
        .Cin(1'b1), // carry igual a 1
        .Sum(diff_out), // asignar a la resta de CLA la resta de la ALU, es decir Sum
                       // "empuja" (da el valor) de la resta realizada a diff_out
        .Cout(diff_carry)
    );
    
    // bloque combinacional
    always @* begin
        Y = 8'h00; // definir Y, resultado con 8 bits, valores por defecto = 0
        C = 1'b0;  // definir carry out salida 1 bit, valore por defecto = 0
        V = 1'b0;  // definir salida overflow 1 bit, valor por defecto 0

        case (op)
            3'b000: begin // suma
                Y = sum_out;   // se asigna el resultado de la suma a la salida y
                C = sum_carry; // se asigna el carry de la suma
                V = A[7] ~^ B[7] ? 1'b0 : (A[7] ^ Y[7]); 
                // overflow solo si A y B tienen el mismo signo y difiere el signo del resultado
                // overflow si el signo de Y difiere del signo de A/B
                // V = (A[7] != Y[7]);
                // si no, V=0
            end

            3'b100: begin // resta (A - B)
                Y = diff_out; // se asgina el resultado de la resta a la salida
                C = diff_carry; // se asigna el carry out si hay
                V = (A[7] ^ B[7]) & (A[7] ^ Y[7]);
                // (A[7] ^ B[7]) detecta que A y B tenían signos distintos.
                // (A[7] ^ Y[7]) detecta que el resultado Y cambió de signo respecto a A.
                // solo si ambas condiciones son verdaderas -> V = 1. 
            end
            // operacion AND y carry = 0
            3'b010: begin Y = A & B; C = 1'b0; end
            
            // operacion OR y carry = 0
            3'b110: begin Y = A | B; C = 1'b0; end
            
            // shift a la derecha logico 1 posicion
            3'b001: begin C = S[0]; Y = S >> 1; end
            
            // shift a la izquierda logico 1 posicion
            3'b101: begin C = S[7]; Y = S << 1; end
            
            // shift a la derecha aritmetico
            3'b011: begin C = S[0]; Y = $signed(S) >>> 1; end
            
            // sin operacion, solo pasa S (numero seleccionado A o B)
            3'b111: begin Y = S; C = 1'b0; end
            
            // caso no valido Y = 0 y Carry  = 0
            default: begin Y = 8'h00; C = 1'b0; end
        endcase
        
        // flags de cero o negativo
        Z = (Y == 8'b0);
        N = Y[7];
    end
       
endmodule

```
## Carry Look-Ahead
Honestamente en mi implementacion debia utilizar el preffix adder, pero no tenia mucha idea de como comenzarlo. En este caso implemente al menos el segundo de mayor dificultad el cual es el carry look-ahead adder. Este se implenta como un submodulo de top, el cual es luego importado a "main" o en este caso a nuestro "top" por medio de la linea:

- `include "cla_8bits.sv"

La idea es poder tener el sumador diferenciado para en caso de ser necesario, modificar y depurar con mas facilidad. Otra parte clave, es que al tenerlo separado podemos hacer una asignacion con los constrains separada. En este caso las entradas son los switches, estas estan directamente relacionadas con el top level, el codigo anterior y de top level, se conectan al submodulo que es el sumador, cla_8bits.

De manera simplificada, mis entradas van:
- De switch 0 a 7, es el numero A.
- De swithc 8 a 15 es el numero B.
- De igual forma la salida es observable en los leds del 0 al 8.
```
module cla_8bits(
    input  wire [7:0] A, // entrada A de 8bits
    input  wire [7:0] B, // entrada b de 8bits
    input  wire       Cin, // entrada carry 1bit
    output wire [7:0] Sum, // salida de suma total
    output wire       Cout // salida de carryout
 );   
    wire [7:0] G; // generate
    wire [7:0] P; // propagate
    wire [8:0] C; // cadena de carries internos
    
    //    G[i] = A[i] & B[i] -> genera (detecta) un carry en la posición i
    //    P[i] = A[i] ^ B[i] -> propaga un carry existente
    assign G = A & B; // calcular generate, A*B
    assign P = A ^ B; // calcular propagate, A+B
    
    // se inicia el carry con un carry externo
    assign C[0] = Cin;
    //    C[i+1] = G[i] | (P[i] & C[i])
    //    si el bit i genera carry, o si lo propaga y venia de c[i].
    assign C[1] = G[0] | (P[0] & C[0]); // generacion de carry o propagacion de uno
    assign C[2] = G[1] | (P[1] & C[1]); // cada linea hace lo mismo en en su [i] bit
    assign C[3] = G[2] | (P[2] & C[2]);
    assign C[4] = G[3] | (P[3] & C[3]);
    assign C[5] = G[4] | (P[4] & C[4]);
    assign C[6] = G[5] | (P[5] & C[5]);
    assign C[7] = G[6] | (P[6] & C[6]);
    assign C[8] = G[7] | (P[7] & C[7]);
    //    sum[i] = p[i] ^ C[i]
    //    porque p = A+B, y se suma el carry entrante a cada bit
    assign Sum = P ^ C[7:0]; // calculo de suma del CLA
    
    assign Cout = C[8]; // carry out, es el ultimo y por eso c[8]
endmodule
```
## Constraints para basys 3 - ALU
En este caso los contrains pues no son algo tan complejo. Decidi hacerlo a la inversa y asignar mis variables a los constrains y no de manera inversa. En mi opinion se me facilita y es mas sencillo dado que el programa ya esta hecho, no necesito reescribir variable por variable.

De ese modo al principio del archivo .xdg que es el siguiente, hay un pequeno resumen de como se asignan los constrains.
```
## basys-3 pinout
##   - SW[0..7]  -> A[0..7]
##   - SW[8..15] -> B[0..7]
##   - btnC      -> sh_sel
##   - btnU,btnL,btnR -> op[0],op[1],op[2]
##   - LED[0..7] > Y[0..7]
##   - LED[8]    > C
##   - LED[9]    > Z
##   - LED[10]   > N
##   - LED[11]   > V

##  Switches para A[7:0] (SW0–SW7)
#set_property PACKAGE_PIN V17 [get_ports {sw[0]}]
set_property PACKAGE_PIN V17 [get_ports {A[0]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {A[0]}]

#set_property PACKAGE_PIN V16 [get_ports {sw[1]}]
set_property PACKAGE_PIN V16 [get_ports {A[1]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {A[1]}]

#set_property PACKAGE_PIN W16 [get_ports {sw[2]}]
set_property PACKAGE_PIN W16 [get_ports {A[2]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {A[2]}]

#set_property PACKAGE_PIN W17 [get_ports {sw[3]}]
set_property PACKAGE_PIN W17 [get_ports {A[3]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {A[3]}]

#set_property PACKAGE_PIN W15 [get_ports {sw[4]}]
set_property PACKAGE_PIN W15 [get_ports {A[4]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {A[4]}]

#set_property PACKAGE_PIN V15 [get_ports {sw[5]}]
set_property PACKAGE_PIN V15 [get_ports {A[5]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {A[5]}]

#set_property PACKAGE_PIN W14 [get_ports {sw[6]}]
set_property PACKAGE_PIN W14 [get_ports {A[6]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {A[6]}]

#set_property PACKAGE_PIN W13 [get_ports {sw[7]}]
set_property PACKAGE_PIN W13 [get_ports {A[7]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {A[7]}]

## Switches para B[7:0] (SW8–SW15)
#set_property PACKAGE_PIN V2 [get_ports {sw[8]}]
set_property PACKAGE_PIN V2  [get_ports {B[0]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {B[0]}]

#set_property PACKAGE_PIN T3 [get_ports {sw[9]}]
set_property PACKAGE_PIN T3  [get_ports {B[1]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {B[1]}]

#set_property PACKAGE_PIN T2 [get_ports {sw[10]}]
set_property PACKAGE_PIN T2  [get_ports {B[2]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {B[2]}]

#set_property PACKAGE_PIN R3 [get_ports {sw[11]}]
set_property PACKAGE_PIN R3  [get_ports {B[3]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {B[3]}]

#set_property PACKAGE_PIN W2 [get_ports {sw[12]}]
set_property PACKAGE_PIN W2  [get_ports {B[4]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[12]}]
set_property IOSTANDARD LVCMOS33 [get_ports {B[4]}]

#set_property PACKAGE_PIN U1 [get_ports {sw[13]}]
set_property PACKAGE_PIN U1  [get_ports {B[5]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {B[5]}]

#set_property PACKAGE_PIN T1 [get_ports {sw[14]}]
set_property PACKAGE_PIN T1  [get_ports {B[6]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[14]}]
set_property IOSTANDARD LVCMOS33 [get_ports {B[6]}]

#set_property PACKAGE_PIN R2 [get_ports {sw[15]}]
set_property PACKAGE_PIN R2  [get_ports {B[7]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {sw[15]}]
set_property IOSTANDARD LVCMOS33 [get_ports {B[7]}]

## Botones (activos-bajo) para sh_sel y op[2:0]
## btnC -> sh_sel
#set_property PACKAGE_PIN U18 [get_ports btnC]
set_property PACKAGE_PIN U18 [get_ports sh_sel]
set_property IOSTANDARD LVCMOS33 [get_ports sh_sel]

## btnU -> op[0]
#set_property PACKAGE_PIN T18 [get_ports btnU]
set_property PACKAGE_PIN T18 [get_ports {op[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {op[0]}]

## btnL -> op[1]
#set_property PACKAGE_PIN W19 [get_ports btnL]
set_property PACKAGE_PIN W19 [get_ports {op[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {op[1]}]

## btnR -> op[2]
#set_property PACKAGE_PIN T17 [get_ports btnR]
set_property PACKAGE_PIN T17 [get_ports {op[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {op[2]}]

## Leds para Y[7:0] (Led0–Led7)
#set_property PACKAGE_PIN U16 [get_ports {led[0]}]
set_property PACKAGE_PIN U16 [get_ports {Y[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {Y[0]}]

#set_property PACKAGE_PIN E19 [get_ports {led[1]}]
set_property PACKAGE_PIN E19 [get_ports {Y[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {Y[1]}]

#set_property PACKAGE_PIN U19 [get_ports {led[2]}]
set_property PACKAGE_PIN U19 [get_ports {Y[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {Y[2]}]

#set_property PACKAGE_PIN V19 [get_ports {led[3]}]
set_property PACKAGE_PIN V19 [get_ports {Y[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {Y[3]}]

#set_property PACKAGE_PIN W18 [get_ports {led[4]}]
set_property PACKAGE_PIN W18 [get_ports {Y[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {Y[4]}]

#set_property PACKAGE_PIN U15 [get_ports {led[5]}]
set_property PACKAGE_PIN U15 [get_ports {Y[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {Y[5]}]

#set_property PACKAGE_PIN U14 [get_ports {led[6]}]
set_property PACKAGE_PIN U14 [get_ports {Y[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {Y[6]}]

#set_property PACKAGE_PIN V14 [get_ports {led[7]}]
set_property PACKAGE_PIN V14 [get_ports {Y[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {Y[7]}]

## leds para flags (Led8–Led11)
## C  -> LED8  (led[8] Carry out)
#set_property PACKAGE_PIN V13 [get_ports {led[8]}]
set_property PACKAGE_PIN V13 [get_ports C]
set_property IOSTANDARD LVCMOS33 [get_ports C]

## Z  -> LED9  (led[9]) Zero
#set_property PACKAGE_PIN V3  [get_ports {led[9]}]
set_property PACKAGE_PIN V3  [get_ports Z]
set_property IOSTANDARD LVCMOS33 [get_ports Z]

## N   Led10 (led[10]) Negative
#set_property PACKAGE_PIN W3  [get_ports {led[10]}]
set_property PACKAGE_PIN W3  [get_ports N]
set_property IOSTANDARD LVCMOS33 [get_ports N]

## V  -> LED11 (led[11]) overflow
#set_property PACKAGE_PIN U3  [get_ports {led[11]}]
set_property PACKAGE_PIN U3  [get_ports V]
set_property IOSTANDARD LVCMOS33 [get_ports V]
```
## Capturas del diseno de la implementacion de la ALU:
![implmentacion alu 2](https://github.com/user-attachments/assets/0094424a-26f7-4487-9310-71d9c555b84d)
![implmentacion alu 3](https://github.com/user-attachments/assets/f3170572-08c0-469a-95d5-ad7194a7585a)





# 2) FSM - Aspiradora

## Link del video del funcionamiento de la aspiradora, con el codigo cargado a la basys 3: [FPGA - Aspiradora](https://youtu.be/veOmlCN_qPY)

Diagrama de logica de funcionamiento de la aspiradora - solo es tipo Moore.
![diagramafsmaspiradora](https://github.com/user-attachments/assets/303d78b7-00e0-49b2-a5a9-f95840b57a2d)

Este codigo es mucho mas sencillo, empezamos por lo que es nuetro top level. Basicamente aqui lo unico que se lleva a cabo es la conexion de los constrains [el hardaware fisico], con nuestras variables. Bien se asignan los constrains a variables, pero luego estos son nuevamente instanciados a nuestro submodulo, que es donde la aspiradora (fsm) realmente funciona.

De igual forma y de manera mas prescisa en este caso se utiliza un clock interno del fpga a 100Mhz, este servira para detectar los flancos de subida de los "sensores" (switches), lo cual no evita usar detecciones como "high" o "low", que harian de una instruccion algo repetitivo.
```
`timescale 100ns / 1ps

module top_fsm_aspiradora (
    input  wire CLK100MHZ,   // reloj a 100 MHz
    input  wire [3:0] SW,    // SW0=power_off, SW1=on, SW2=cleaning, SW3=evading
    output wire [1:0] led    // led0 y led1 muestran el estado actual
);

    // renombramos para claridad
    // solo les damos nombres a cada i del array de los switches
    wire power_off  = SW[0];
    wire on         = SW[1];
    wire cleaning   = SW[2];
    wire evading    = SW[3];

    // conexion, canal, para ver el estado de la
    // aspiradora
    logic [1:0] state;

    // instanciamos la fsm (las "opoeraciones"), y exponemos solo state
    FSM_Aspiradora (
        .clk        (CLK100MHZ),
        .power_off  (power_off),
        .on         (on),
        .cleaning   (cleaning),
        .evading    (evading),
        .state_0    (state)
    );

    // conectamos los leds 0 y 1 con el estado en que se encuentran
    assign led = state;
endmodule
```
## Sub-modulo de la fsm - Aspiradora

En este modulo se ejecutan las operaciones que la aspiradora realizara. En este caso hay un detalle importante y es que al usar un clock, es importante definir por ejemplo, en que momentos se captan los cambios de estado. Es decir que, los switches simulan ser mis "sensores", y dado que los muevo de forma manual, es conveniente no tener rebotes o bien, operaciones repetitivas. Para esto se emplea la detecion del flanco positivo, ya que si se utiliza la deteccion de un "high" o "low" estado alto o bajo, este seria repetitivo en todos los ciclos, a diferencia un flanco positivo seria cuestion de una sola deteccion (one-shot). Y aunque el boton o switch queden en estado alto o bajo, no hara que la operacion sea repetitiva y da lugar a que se puedan captar nuevos estados mas rapidamente.

```
// definicion de entradas y salidas, al ser "semi-automatica" no requiere salidas en realidad.
module FSM_Aspiradora  (input logic clk,
                        input logic power_off,
                        input logic on,
                        input logic cleaning,
                        input logic evading,
                        output logic [1:0] state_0); // Solo para visualizar el estado.
// se definen y enumeran los estados                        
typedef enum logic [1:0] {off_s, exploring_s, cleaning_s, evading_s} state_type;

// registro de estados
state_type state, next_state;

// registro de estado de power_off, logica secuencial a base de flip-flops
always_ff @(posedge clk or posedge power_off) begin // posedge -> flanco positivo.
                                           // begin y end
                                           // sirven para delimitar un bloque de sentencias.
    if(power_off) // se fuerza un reset que es asincrono, este puede darse en cualquier
        state <= off_s;// momento de la "ejecucion".
    else 
        state <= next_state; // si no hay orden de apagado, se carga el next state en cada cambio de flanco
end                          // positivo del reloj.

// la logica combinacional del cambio de estados:

always_comb begin // bloque de logica combinacional.   
    next_state = state; // si no hay cambios, se queda en el estado actual.    
    case (state)  
          
        off_s: begin
            if (on)
            // solo se sale de off si on se activa y automaticamente ira a explorar.
                next_state = exploring_s;
            else 
            // si on no se activa, la aspiradora se quedaria en reposo.
                next_state = off_s;
        end
        
        exploring_s: begin // mientras la aspiradora explora tiene 3 posibles estados que le sigan:
            if (power_off) // si hay senal de apagado, pues la aspiradora se apaga.
                next_state = off_s;
            else if (cleaning)  // si hay senal de suciedad, pues la aspiradora comenzara a limpiar.
                next_state = cleaning_s;
            else if (evading)// si hay senal de obstaculo, pues esquiva el obstaculo.
                next_state = evading_s;
            else // si no se da uno de los casos anteriores, la aspiradora continua explorando.
                next_state = exploring_s;
        end
        
        cleaning_s: begin // es el bloque para los cambios de estado mientras "aspira"
            if (power_off) // si hay senal de apagado, pues la aspiradora se apaga.
                next_state = off_s;
            else if (!cleaning) // si no hay senal de suciedad, la aspiradora regresa a explorar.
                next_state = exploring_s; 
            else // si no hay cambios en los inputs, la aspiradora sigue limpiando (aspirando).
                next_state = cleaning_s;
            end
            
        evading_s: begin
            if (power_off) // si hay senal de apagado, la aspiradora se apaga.
                next_state = off_s;
            else if (!evading) // si deja de haber obstaculo, la aspiradora continua explorando.
                next_state = exploring_s;
            else // si no hay cambios en los inputs, la aspiradora continuara explorando.
                next_state = evading_s;
            end
    endcase // se terminan los posibles casos o mas bien, casos de cambios de estado.
end // termina bloque de logica secuencial.

assign state_0 = state; // se asigna el estado actual a la variable para visualizarlo. No tienen otro proposito.

endmodule
```
## Constraints basys 3 - Aspiradora:
En este aspecto no es muy compilcado. Las entradas y salidas son relativamente pocas. Lo mas complejo en esta seccion, seria el reloj. De maner un poco detallada funciona asi:

- set_property PACKAGE_PIN W5   [get_ports CLK100MHZ] : Se asigna un pin fisico al reloj de 100Mhz en vivado para que este sepa de donde viene el reloj externo. Ese pin es el W5 el cual va soldado directamente al reloj de 100Mhz.
- create_clock -period 10.000 -name sys_clk_pin -waveform {0 5} [get_ports CLK100MHZ] :
  - period 10.000: el tiempo entre dos flancos de subida consecutivos (un ciclo completo) es de 10 ns.
  - name sys_clk_pin: le da un nombre interno al objeto de reloj, para referirlo en otras restricciones de timing si es necesario.
  - waveform {0 5}: define que el flanco de subida ocurre en el instante “0 ns” y el de bajada en “5 ns”. Así Vivado entiende dónde están los flancos de subida/bajada dentro de cada ciclo.

[get_ports CLK100MHZ]: aplica esta definición de reloj a la señal interna CLK100MHZ (que ya fue atada al pin W5).
```
## reloj 100 MHz
set_property PACKAGE_PIN W5   [get_ports CLK100MHZ]
set_property IOSTANDARD LVCMOS33 [get_ports CLK100MHZ]
create_clock -period 10.000 -name sys_clk_pin -waveform {0 5} [get_ports CLK100MHZ]

## switches como entradas de los sensores
# SW0 power_off
set_property PACKAGE_PIN V17  [get_ports {SW[0]}]
# SW1 on
set_property PACKAGE_PIN V16  [get_ports {SW[1]}]
# SW2 cleaning
set_property PACKAGE_PIN W16  [get_ports {SW[2]}]
# SW3 evading
set_property PACKAGE_PIN W17  [get_ports {SW[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports SW[*]]

## leds muestran el estado (2 bits) ver diagrama
# led0 state[0]
set_property PACKAGE_PIN U16  [get_ports {LED[0]}]
# led1 state[1]
set_property PACKAGE_PIN E19  [get_ports {LED[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports LED[*]]
```

## Capturas de la implementacion de la fsm-Aspiradora:
![implmentacion fsm 1](https://github.com/user-attachments/assets/b8386fa4-2188-492c-a06f-c769569e1106)
![implmentacion fsm 2](https://github.com/user-attachments/assets/480add19-fa40-4e45-aa2a-c383541b86ad)



# Flujo de trabajo de Vivado, Como funciona?

1) Elaboración (Elaboration):

Vivado lee el codigo HDL (Verilog/VHDL), comprueba consistencia de tipos y puertos, expande los generate y parámetros, resuelve instanciaciones de módulos y construye un netlist lógico. Aquí se valida que el diseño sea sintáctica y estructuralmente correcto antes de llevarlo a síntesis.


2) Síntesis (Synthesis)

Traducir el netlist lógico (puertas AND/OR, flops, sumadores, etc.) a una red de primitivas específicas de la FPGA: LUTs, flip-flops, carry-chains, bloques de RAM, DSPs, etc.

- Optimización lógica: Consolida lógica redundante, minimiza expresiones booleanas y agrupa operadores de igual función.
- Asignación de recursos: Decide qué lógica va a ir en LUTs, cuáles aprovecharán los carry-chains, y dónde usará flip-flops integrados.
- Mapeo: Convierte cada puerta lógica a LUTs de 6 entradas (en Virtex-7 o Artix-7, por ejemplo). Cuando una operación XOR de 8 bits se descompone, Vivado la reparte en varias LUTs.


3) Optimizacion post Sintesis:
- Optimización de ruta del carry: Vivado detecta estructuras típicas de sumadores y las asocia a los carry-chains (un recurso especializado que conecta la salida de carry de una LUT a la siguiente con muy baja latencia).
- Retiming: Mueve flops a lo largo de la ruta de datos para balancear retardos y mejorar frecuencia máxima.


4) Generación de netlist tecnologico:
El resultado de síntesis es un netlist que nombra instancias de LUTs, FFs, carry slicers, RAMB18, DSP48, etc., listo para colocar en la FPGA.


5) Implementación (Implementation)
- Translate & Assemble: Combina netlists y chequea constraints de pines y de timing.

- Map: Asigna cada instancia tecnológica a un slice físico de la FPGA.
  - Los slices son unidades dentro del CLB (Configurable Logic Block) que contienen generalmente 4 LUTs de 6 entradas y 8 flip-flops.
  - CLB (Configurable Logic Block): grupo de slices con su propia interconexión interna.

- Place: Decide en qué CLB y slice entra cada LUT y cada FF, intentando minimizar rutas largas y congestionamiento.
  - Se tienen en cuenta:
  - Constraints de pinout (.xdc) para ubicar señales de I/O en pines específicos.
  - Timing constraints (SDC) para cumplir requisitos de frecuencia.

- Route: Trazado de las interconexiones (routing) por la matriz de switchboxes y canales de la FPGA. Vivado busca rutas de baja congestión y que cumplan con los márgenes de timing.


6) Timing Analysis: 
- Se calculan retardos de cada camino crítico (desde flops o entradas a salidas o flops).
- Si algo excede la meta de frecuencia, Vivado reporta violaciones de timing y propone optimizaciones.


7) Bitstream Generation:
- Finalmente elabora el archivo binario (.bit) que contiene la configuración de todas las LUTs, routing multiplexers y recursos especiales de la FPGA.

# Papel de los Constraints:
- Mapeo de los puertos HDL, a pines físicos (PACKAGE_PIN V17, U18, U16…).
- IOSTANDARD define tensión y características eléctricas (LVCMOS33, etc.).

- Sin un XDC completo, la FPGA no sabrá dónde “soldar” cada señal.

- Timing Constraints (.sdc opcional)
  - Especificar tu frecuencia objetivo (Por ejemplo al asignar un clock: create_clock –period 10ns es el periodo –name clk es el pin).
  - Definir entradas asíncronas, false paths o multicycle paths.

