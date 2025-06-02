# Examen-Final FPGA-implementacion

# 1) Alu de 8 bits

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
        .A(A), // asignar a A CLA, el A del input de la ALU
        .B(B), // lo mismo para B, a CLA el input de la ALU
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
## Constrains Para basys 3
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
## Algunas fotos de la implementacion:
![implmentacion alu 1](https://github.com/user-attachments/assets/f278b7ae-6dbb-4811-9fde-24f4163c57d3)
![implmentacion alu 2](https://github.com/user-attachments/assets/0094424a-26f7-4487-9310-71d9c555b84d)
![implmentacion alu 3](https://github.com/user-attachments/assets/f3170572-08c0-469a-95d5-ad7194a7585a)



