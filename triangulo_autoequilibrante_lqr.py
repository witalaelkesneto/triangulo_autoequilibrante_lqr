import serial
import csv
import time
from datetime import datetime

# ===================== CONFIGURAÇÕES =====================

PORTA_SERIAL = "COM3"
BAUDRATE = 115200
TEMPO_COLETA = 40

data_hora = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
NOME_ARQUIVO = f"dados_planta_{data_hora}.csv"

# ===================== FUNÇÕES AUXILIARES =====================

def linha_numerica_valida(linha):
    dados = linha.split(",")

    if len(dados) != 7:
        return False

    try:
        int(float(dados[0]))
        float(dados[1])
        float(dados[2])
        float(dados[3])
        float(dados[4])
        float(dados[5])
        float(dados[6])
        return True
    except ValueError:
        return False


def tentar_reset_esp32(serial_esp):
    print("Tentando reiniciar/liberar ESP32...")

    serial_esp.dtr = False
    serial_esp.rts = False
    time.sleep(0.5)

    serial_esp.dtr = True
    serial_esp.rts = True
    time.sleep(0.2)

    serial_esp.dtr = False
    serial_esp.rts = False
    time.sleep(4.5)

    print("ESP32 liberado. Aguardando dados...")


# ===================== COLETA =====================

def main():
    print("Iniciando coleta de dados...")
    print(f"Porta: {PORTA_SERIAL}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Tempo de coleta: {TEMPO_COLETA} s")
    print(f"Arquivo: {NOME_ARQUIVO}")

    try:
        serial_esp = serial.Serial(
            PORTA_SERIAL,
            BAUDRATE,
            timeout=1,
            dsrdtr=False,
            rtscts=False
        )

        tentar_reset_esp32(serial_esp)

        print("Aguardando início dos dados numéricos...")

        with open(NOME_ARQUIVO, mode="w", newline="", encoding="utf-8") as arquivo:
            escritor = csv.writer(arquivo)

            escritor.writerow([
                "tempo_s",
                "modo",
                "theta_rad",
                "theta_dot_rad_s",
                "omega_rad_s",
                "xi_rad",
                "energia_J",
                "u_V"
            ])

            coleta_iniciada = False
            tempo_inicial = None
            tempo_espera = time.time()

            while True:
                linha = serial_esp.readline().decode("utf-8", errors="ignore").strip()

                if not linha:
                    if not coleta_iniciada and (time.time() - tempo_espera > 15):
                        print("\nNenhum dado numérico foi recebido após 15 segundos.")
                        print("Pressione o botão EN/RESET do ESP32 com este programa ainda rodando.")
                        print("Se ainda não aparecer nada, confira se o Monitor Serial da Arduino IDE está fechado.")
                        tempo_espera = time.time()
                    continue

                print(f"Serial: {linha}")

                if not linha_numerica_valida(linha):
                    continue

                if not coleta_iniciada:
                    coleta_iniciada = True
                    tempo_inicial = time.time()
                    print("\nDados numéricos detectados.")
                    print("Coleta iniciada.\n")

                tempo_decorrido = time.time() - tempo_inicial

                if tempo_decorrido >= TEMPO_COLETA:
                    break

                dados = linha.split(",")

                modo = int(float(dados[0]))
                theta = float(dados[1])
                theta_dot = float(dados[2])
                omega = float(dados[3])
                xi = float(dados[4])
                energia = float(dados[5])
                u = float(dados[6])

                escritor.writerow([
                    tempo_decorrido,
                    modo,
                    theta,
                    theta_dot,
                    omega,
                    xi,
                    energia,
                    u
                ])

                print(
                    f"t={tempo_decorrido:.3f}s | "
                    f"modo={modo} | "
                    f"theta={theta:.4f} rad | "
                    f"theta_dot={theta_dot:.4f} rad/s | "
                    f"omega={omega:.4f} rad/s | "
                    f"xi={xi:.4f} rad | "
                    f"E={energia:.4f} J | "
                    f"u={u:.4f} V"
                )

        serial_esp.close()

        print("\nColeta finalizada.")
        print(f"Dados salvos em: {NOME_ARQUIVO}")

    except Exception as e:
        print("Erro ao abrir ou ler a porta serial.")
        print(e)
        print("Verifique se a porta está correta e se o Monitor Serial do Arduino está fechado.")


if __name__ == "__main__":
    main()