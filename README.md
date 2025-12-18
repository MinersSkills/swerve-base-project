# FRC Base Robot Project

Projeto base de **robô FRC** desenvolvido para servir como fundação sólida e reutilizável para as **próximas temporadas**. Este repositório foi pensado para ser **modular, escalável e fácil de manter**, seguindo as melhores práticas recomendadas pela **WPILib**.

---

## Padrão de Commits

Este repositório segue um padrão simples de commits para manter o histórico organizado:

* `feat:` nova funcionalidade para o robô
* `chore:` nova pequena funcionalidade ou ajuste fino
* `fix:` correção de algum problema

---

## Visão Geral

Este projeto utiliza a arquitetura **Command-Based**, com foco em:

* Código limpo e organizado
* Separação clara de responsabilidades
* Facilidade para adicionar novos subsistemas e comandos
* Reutilização entre temporadas

Atualmente, o projeto já conta com:

* **Swerve Drive totalmente funcional (utilizando **YAGSL**)
* **Estimação de pose com visão (Limelight)** integrada ao odometry
* **Comando Drive To Pose funcionando** (movimento autônomo até uma pose alvo)
* Suporte a PathPlanner

---

## Arquitetura do Projeto

A arquitetura segue o padrão **Command-Based** da WPILib:

```
Robot
 ├── RobotContainer
 │    ├── Subsystems
 │    └── Commands
 │ 
 ├── Commands
 ├── Subsystems
 └── Constants
```

### Por que Command-Based?

* Facilita manutenção
* Permite paralelismo entre comandos
* Escala bem conforme o robô cresce
* Padrão oficial recomendado pela FIRST

---

## Estrutura de Pastas

### `src/main/deploy`

Contém arquivos de configuração utilizados no runtime do robô.

```
deploy/
 ├── pathplanner/        # Paths e autos do PathPlanner
 └── swerve/
     └── neo/
         ├── modules/    # Configuração individual dos módulos
         │   ├── frontleft.json
         │   ├── frontright.json
         │   ├── backleft.json
         │   └── backright.json
         ├── physicalproperties.json
         ├── pidfproperties.json
         ├── controllerproperties.json
         └── swervedrive.json
```

Esses arquivos permitem ajustes finos de **PID, geometria, motores e offsets** sem recompilar o código.

---

### `src/main/java/frc/robot`

#### `subsystems/`

* **SwerveSubsystem.java**
  Responsável por todo o controle do drivetrain swerve:

  * Cinemática
  * Odometry
  * Integração com visão

---

#### `commands/swervedrive/drivebase/`

Comandos de controle do robô:

* `AbsoluteDrive.java` – Drive absoluto
* `AbsoluteDriveAdv.java` – Versão avançada
* `AbsoluteFieldDrive.java` – Drive field-oriented
* **`DriveToPose.java`** – Move o robô automaticamente até uma pose alvo no campo

---

#### `limelight/`

Estimação de pose usando visão:

* `LimelightHelpers.java` – Interface com a Limelight
* `LimelightPoseEstimator.java` –

  * Fusão de visão + odometry
  * Valida medições
  * Atualiza pose global do robô

---

#### `generalconstants/`

* `DriveToPoseConstants.java`
  Constantes específicas para o controle do Drive To Pose (PID, tolerâncias, etc).

---

#### Arquivos principais

* `Constants.java` – Constantes globais
* `Robot.java` – Ciclo de vida do robô
* `RobotContainer.java` –

  * Inicialização dos subsistemas
  * Mapeamento de controles
  * Registro de comandos
* `Main.java` – Entry point

---

## Estimação de Pose com Visão

O projeto já possui **estimação de pose funcional**, utilizando:

* Odometry do Swerve
* Dados de visão da **Limelight**
* Filtros e validações para rejeitar medições ruins

Isso permite:

* Pose global mais precisa
* Autônomos mais confiáveis
* Uso eficiente do Drive To Pose

---

## Drive To Pose

O comando **DriveToPose** permite que o robô:

* Vá automaticamente até uma posição desejada
* Controle X, Y e rotação
* Utilize PID e tolerâncias configuráveis

Ideal para:

* Autônomos
* Alinhamento automático
* Ações assistidas no teleop

---

## PathPlanner

Suporte nativo a **PathPlanner**, permitindo:

* Criação visual de trajetórias
* Autos reutilizáveis
* Integração direta com o swerve

Arquivos ficam em:

```
src/main/deploy/pathplanner
```

---

## Objetivo do Projeto

Este repositório foi criado para ser:

* Uma **base reutilizável** para futuras temporadas
* Fácil de adaptar a novos jogos
* Escalável conforme o robô evolui
* Um projeto de referência para a equipe

---

## Próximos Passos

* [ ] Implementação de superstructres para melhor controle dos subsistemas.
