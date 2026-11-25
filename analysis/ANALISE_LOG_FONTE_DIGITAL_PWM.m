% ANALISE_LOG_FONTE_DIGITAL_PWM.m
% Analise do log da fonte digital 0 a 10 V com controle deadbeat.
% Le o arquivo de texto copiado do monitor serial (nao abre a porta serial),
% identifica um modelo de 1a ordem, valida o modelo, gera Bode, Nyquist,
% respostas ao degrau e o desempenho de cada degrau, e exporta figuras (PNG)
% e tabelas (CSV).
% Requer Control System Toolbox e System Identification Toolbox.

clear; clc; close all;

% Configuracao
ARQUIVO_LOG       = "log_teste1.txt";
PASTA_FIGURAS     = "figuras_relatorio";
EXPORTAR_FIGURAS  = true;

if EXPORTAR_FIGURAS && ~isfolder(PASTA_FIGURAS)
    mkdir(PASTA_FIGURAS);
end

aplicaEstiloPadrao();

fprintf('=========================================================\n');
fprintf('  ANALISE DE LOG - FONTE DIGITAL PWM (Controle Deadbeat)\n');
fprintf('  (somente leitura do arquivo de log)\n');
fprintf('=========================================================\n\n');

% 1) Importacao do log
fprintf('[1/8] Importando "%s"...\n', ARQUIVO_LOG);

if ~isfile(ARQUIVO_LOG)
    error('Arquivo "%s" não encontrado. Coloque o log nessa pasta e ajuste ARQUIVO_LOG.', ARQUIVO_LOG);
end

[dados, eventos] = importaLog(ARQUIVO_LOG);
dados = sortrows(dados, 't_ms');
dados.t_s = (dados.t_ms - dados.t_ms(1)) / 1000;

fprintf('      %d amostras de dados importadas (%.1fs de log).\n', height(dados), dados.t_s(end));
if ~isempty(eventos)
    fprintf('      %d eventos (EVT) encontrados no log detalhado.\n', height(eventos));
    writetable(eventos, 'log_eventos_completo.csv');
    resumoEventos(eventos);
end
fprintf('\n');

t  = dados.t_s;
u  = dados.u;
y  = dados.y;
SP = dados.SP;

% 2) Identificacao de um modelo G(s) = K/(tau s + 1)
fprintf('[2/8] Identificando o sistema (modelo de 1a ordem G(s)=K/(tau.s+1))...\n');

Ts_ident = 0.05;
tUnif = (0:Ts_ident:t(end))';
% Reamostragem com retencao de ordem zero: o comando fica constante entre amostras
uUnif = interp1(t, u, tUnif, 'previous', 'extrap');
yUnif = interp1(t, y, tUnif, 'previous', 'extrap');

dadosID = iddata(yUnif, uUnif, Ts_ident);
dadosID.TimeUnit = 'seconds';
dadosID.InputName  = 'u (comando, V)';
dadosID.OutputName = 'y (tensao medida, V)';

opcoesFit = tfestOptions('Display', 'off');
G_ident = tfest(dadosID, 1, 0, opcoesFit);

K_ident   = dcgain(G_ident);
poloIdent = pole(G_ident);
tau_ident = -1 / real(poloIdent(1));
fitPct    = G_ident.Report.Fit.FitPercent;

fprintf('      K estimado   = %.4f   (firmware usa Kgain = 1.0; ganho real medido ~1.01)\n', K_ident);
fprintf('      tau estimado = %.4f s (%.1f ms)   (assumido no firmware: 30 ms)\n', tau_ident, tau_ident*1000);
fprintf('      Ajuste do modelo (fit) = %.2f%%\n\n', fitPct);

% Modelo de referencia: tau do firmware e ganho medido da planta (o firmware usa K = 1)
K_firmware   = 1.01;
tau_firmware = 0.030;
G_firmware   = tf(K_firmware, [tau_firmware, 1]);

% 3) Validacao do modelo identificado
fprintf('[3/8] Validando modelo identificado contra o dado real...\n');
fprintf('      O fit usa NRMSE, que penaliza pequenas defasagens nas bordas dos degraus.\n\n');

figValidacao = figure('Name', 'Validacao do Modelo Identificado', 'Position', [80 80 900 500]);
compare(dadosID, G_ident);
title('Validação do modelo identificado (1ª ordem)', 'FontWeight', 'bold');
xlabel('Tempo (s)');
ylabel('y -- tensão medida (V)');
legend('Dado medido', 'Modelo identificado', 'Location', 'best');
grid on;
salvaFigura(figValidacao, PASTA_FIGURAS, EXPORTAR_FIGURAS, '03_validacao_modelo');

% 4) Bode e margens
fprintf('[4/8] Gerando diagrama de Bode e margens de estabilidade...\n');

pBode = bodeoptions('cstprefs');
pBode.Title.String  = 'Diagrama de Bode -- planta (módulo PWM -> tensão)';
pBode.XLabel.String = 'Frequência (rad/s)';
pBode.YLabel.String = {'Magnitude (dB)', 'Fase (graus)'};
pBode.Grid = 'on';

figBode = figure('Name', 'Diagrama de Bode', 'Position', [80 80 900 600]);
bodeplot(G_ident, G_firmware, pBode);
legend('Modelo identificado (dados reais)', 'Modelo assumido (firmware)', 'Location', 'best');
salvaFigura(figBode, PASTA_FIGURAS, EXPORTAR_FIGURAS, '04_bode');

[Gm, Pm, Wcg, Wcp] = margin(G_ident);
fprintf('      Margem de ganho  = %.2f dB (em %.3f rad/s)\n', 20*log10(Gm), Wcg);
fprintf('      Margem de fase   = %.2f graus (em %.3f rad/s)\n\n', Pm, Wcp);

% 5) Nyquist
fprintf('[5/8] Gerando diagrama de Nyquist...\n\n');

pNyq = nyquistoptions('cstprefs');
pNyq.Title.String  = 'Diagrama de Nyquist -- planta';
pNyq.XLabel.String = 'Eixo real';
pNyq.YLabel.String = 'Eixo imaginário';
pNyq.Grid = 'on';

figNyquist = figure('Name', 'Diagrama de Nyquist', 'Position', [80 80 700 700]);
nyquistplot(G_ident, G_firmware, pNyq);
legend('Modelo identificado', 'Modelo assumido (firmware)', 'Location', 'best');
salvaFigura(figNyquist, PASTA_FIGURAS, EXPORTAR_FIGURAS, '05_nyquist');

% 6) Respostas ao degrau (stairs mostra o valor mantido entre amostras)
fprintf('[6/8] Gerando gráficos de resposta ao degrau...\n\n');

figDegrau = figure('Name', 'Resposta ao Degrau Completa', 'Position', [80 80 1000 500]);
stairs(t, SP, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4); hold on;
stairs(t, y, '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 1.6);
xlabel('Tempo (s)'); ylabel('Tensão (V)');
legend('Setpoint (referência)', 'y medido (planta + controle deadbeat)', 'Location', 'best');
title('Resposta ao degrau -- sistema completo (malha fechada)', 'FontWeight', 'bold');
grid on;
salvaFigura(figDegrau, PASTA_FIGURAS, EXPORTAR_FIGURAS, '06_resposta_degrau_completa');

pStep = timeoptions('cstprefs');
pStep.Title.String  = 'Resposta ao degrau unitário -- planta em malha aberta';
pStep.XLabel.String = 'Tempo (s)';
pStep.YLabel.String = 'Amplitude';
pStep.Grid = 'on';

figStepPlanta = figure('Name', 'Resposta ao Degrau - Planta (malha aberta)', 'Position', [80 80 900 500]);
stepplot(G_ident, G_firmware, 1, pStep);
legend('Planta identificada', 'Planta assumida (firmware)', 'Location', 'best');
salvaFigura(figStepPlanta, PASTA_FIGURAS, EXPORTAR_FIGURAS, '07_step_planta_malha_aberta');

% 7) Desempenho por degrau
fprintf('[7/8] Calculando métricas de desempenho por degrau...\n');

resultados = calculaMetricasPorDegrau(t, y, SP);
disp(resultados);
writetable(resultados, 'resultados_por_degrau.csv');
fprintf('      Tabela salva em "resultados_por_degrau.csv"\n\n');

figResumo = figure('Name', 'Resumo de Desempenho por Degrau', 'Position', [80 80 1000 600]);

subplot(2,2,1);
bar(resultados.SP_V, resultados.Overshoot_pct, 'FaceColor', [0.85 0.33 0.10]);
xlabel('Setpoint (V)'); ylabel('Overshoot (%)');
title('Overshoot por Degrau'); grid on;

subplot(2,2,2);
bar(resultados.SP_V, resultados.TempoAcomodacao_s, 'FaceColor', [0 0.4470 0.7410]);
xlabel('Setpoint (V)'); ylabel('t_{acomodação} (s)');
title('Tempo de Acomodação por Degrau'); grid on;

subplot(2,2,3);
bar(resultados.SP_V, resultados.TempoSubida_s, 'FaceColor', [0.47 0.67 0.19]);
xlabel('Setpoint (V)'); ylabel('t_{subida} (s)');
title('Tempo de Subida por Degrau'); grid on;

subplot(2,2,4);
bar(resultados.SP_V, resultados.ErroRegime_pct, 'FaceColor', [0.49 0.18 0.56]);
xlabel('Setpoint (V)'); ylabel('Erro de Regime (%)');
title('Erro de Regime Permanente por Degrau'); grid on;

sgtitle('Resumo de Desempenho -- Controle Deadbeat + Trim Integrador', 'FontWeight', 'bold', 'FontSize', 13);
salvaFigura(figResumo, PASTA_FIGURAS, EXPORTAR_FIGURAS, '08_resumo_desempenho');

% 8) Resumo
fprintf('[8/8] Concluído.\n\n');
fprintf('=========================================================\n');
fprintf('  RESUMO EXECUTIVO\n');
fprintf('=========================================================\n');
fprintf('  Modelo identificado : G(s) = %.3f / (%.4f s + 1)\n', K_ident, tau_ident);
fprintf('  Modelo de referencia : G(s) = %.3f / (%.4f s + 1)  (tau do firmware, K medido)\n', K_firmware, tau_firmware);
fprintf('  Ajuste do modelo (fit)         : %.2f%%\n', fitPct);
fprintf('  Margem de ganho                : %.2f dB\n', 20*log10(Gm));
fprintf('  Margem de fase                 : %.2f graus\n', Pm);
fprintf('  Overshoot médio (malha fechada): %.2f%%\n', mean(resultados.Overshoot_pct, 'omitnan'));
fprintf('  Tempo de acomodação médio      : %.2f s\n', mean(resultados.TempoAcomodacao_s, 'omitnan'));
fprintf('  Erro de regime médio           : %.2f%%\n', mean(resultados.ErroRegime_pct, 'omitnan'));
fprintf('=========================================================\n');
if EXPORTAR_FIGURAS
    fprintf('  Figuras exportadas em: %s/\n', PASTA_FIGURAS);
end
fprintf('  Tabela de resultados : resultados_por_degrau.csv\n');
if ~isempty(eventos)
    fprintf('  Log de eventos       : log_eventos_completo.csv\n');
end
fprintf('=========================================================\n');

% Funcoes auxiliares
function aplicaEstiloPadrao()
    set(groot, 'DefaultAxesFontName', 'Helvetica');
    set(groot, 'DefaultAxesFontSize', 11);
    set(groot, 'DefaultTextFontName', 'Helvetica');
    set(groot, 'DefaultAxesBox', 'on');
    set(groot, 'DefaultAxesLineWidth', 1);
    set(groot, 'DefaultFigureColor', 'w');
    set(groot, 'DefaultLineLineWidth', 1.4);
    set(groot, 'DefaultAxesXGrid', 'on');
    set(groot, 'DefaultAxesYGrid', 'on');
end

function salvaFigura(fig, pastaFiguras, exportar, nomeBase)
    if ~exportar
        return;
    end
    caminho = fullfile(pastaFiguras, nomeBase + ".png");
    exportgraphics(fig, caminho, 'Resolution', 300);
end

function resumoEventos(eventos)
    fprintf('\n      === RESUMO DO LOG DE EVENTOS ===\n');
    [categorias, ~, idx] = unique(eventos.categoria);
    contagens = accumarray(idx, 1);
    for k = 1:numel(categorias)
        fprintf('      %-25s : %d\n', categorias{k}, contagens(k));
    end
    erros = eventos(strcmp(eventos.categoria, 'ERRO'), :);
    if ~isempty(erros)
        fprintf('      >>> ATENÇÃO: %d evento(s) de ERRO, ver log_eventos_completo.csv\n', height(erros));
    end
    estabilizacoes = eventos(strcmp(eventos.categoria, 'ESTABILIZOU'), :);
    if ~isempty(estabilizacoes)
        fprintf('      %d degrau(s) estabilizaram, detalhes em log_eventos_completo.csv\n', height(estabilizacoes));
    end
end

% Le linhas de dados em CSV (ou no formato antigo com barras) e linhas de evento EVT
function [dados, eventos] = importaLog(nomeArquivo)
    linhas = readlines(nomeArquivo);
    linhas = strtrim(linhas);
    linhas = linhas(linhas ~= "");

    padraoCSV  = '^(\d+),(\d+),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),([01])$';
    padraoPipe = '^(\d+)\s+(\d+)\s+(\d+)\s+SP:(-?\d+\.?\d*)\s*\|\s*y:(-?\d+\.?\d*)\s*\|\s*u:(-?\d+\.?\d*)\s*\|\s*bias:(-?\d+\.?\d*)';
    padraoEVT  = '^EVT,(\d+),([A-Z_]+),(.*)$';

    t_ms = []; passo = []; SP = []; y = []; u = []; bias = []; estavel = [];
    evt_t = []; evt_categoria = {}; evt_detalhe = {};

    for i = 1:numel(linhas)
        l = linhas(i);
        tk = regexp(l, padraoCSV, 'tokens', 'once');
        if ~isempty(tk)
            t_ms(end+1)    = str2double(tk{1});
            passo(end+1)   = str2double(tk{2});
            SP(end+1)      = str2double(tk{3});
            y(end+1)       = str2double(tk{4});
            u(end+1)       = str2double(tk{5});
            bias(end+1)    = str2double(tk{6});
            estavel(end+1) = str2double(tk{7});
            continue;
        end
        tk = regexp(l, padraoPipe, 'tokens', 'once');
        if ~isempty(tk)
            t_ms(end+1)    = str2double(tk{1});
            passo(end+1)   = str2double(tk{3});
            SP(end+1)      = str2double(tk{4});
            y(end+1)       = str2double(tk{5});
            u(end+1)       = str2double(tk{6});
            bias(end+1)    = str2double(tk{7});
            estavel(end+1) = NaN;
            continue;
        end
        tk = regexp(l, padraoEVT, 'tokens', 'once');
        if ~isempty(tk)
            evt_t(end+1)         = str2double(tk{1});
            evt_categoria{end+1} = tk{2};
            evt_detalhe{end+1}   = tk{3};
        end
    end

    if isempty(t_ms)
        error('Nenhuma linha de dados (CSV ou pipe) reconhecida no log. Confira o arquivo.');
    end

    dados = table(t_ms', passo', SP', y', u', bias', estavel', ...
        'VariableNames', {'t_ms','passo','SP','y','u','bias','estavel'});

    if isempty(evt_t)
        eventos = table();
    else
        eventos = table(evt_t', evt_categoria', evt_detalhe', ...
            'VariableNames', {'t_ms','categoria','detalhe'});
    end
end

% Overshoot, tempo de subida (10 a 90%), acomodacao (faixa de 2%) e erro de regime de cada degrau
function resultados = calculaMetricasPorDegrau(t, y, SP)
    mudancasSP = find(diff(SP) ~= 0) + 1;
    inicioDegraus = [1; mudancasSP];

    resultados = table();
    for i = 1:numel(inicioDegraus)
        idxIni = inicioDegraus(i);
        if i < numel(inicioDegraus)
            idxFim = inicioDegraus(i+1) - 1;
        else
            idxFim = numel(y);
        end

        spAlvo = SP(idxIni);
        yTrecho = y(idxIni:idxFim);
        tTrecho = t(idxIni:idxFim) - t(idxIni);

        if spAlvo == 0 || numel(yTrecho) < 3
            continue;
        end

        valorFinal = yTrecho(end);
        pico = max(yTrecho);
        overshoot_pct = max(0, (pico - spAlvo) / spAlvo * 100);

        % A subida e interpolada entre as amostras vizinhas, porque acontece em menos de uma amostra
        lim10 = 0.10 * spAlvo;
        lim90 = 0.90 * spAlvo;

        [~, idxPico] = max(yTrecho);
        ySubida = yTrecho(1:idxPico);
        tSubida = tTrecho(1:idxPico);
        [ySubidaUnica, ia] = unique(ySubida, 'stable');
        tSubidaUnica = tSubida(ia);

        if numel(ySubidaUnica) >= 2 && max(ySubidaUnica) >= lim90 && min(ySubidaUnica) <= lim10
            t10 = interp1(ySubidaUnica, tSubidaUnica, lim10, 'linear');
            t90 = interp1(ySubidaUnica, tSubidaUnica, lim90, 'linear');
            tempoSubida = t90 - t10;
        else
            tempoSubida = NaN;
        end

        faixa = 0.02 * spAlvo;
        dentroFaixa = abs(yTrecho - spAlvo) <= faixa;
        tempoAcomodacao = NaN;
        for k = 1:numel(dentroFaixa)
            if all(dentroFaixa(k:end))
                tempoAcomodacao = tTrecho(k);
                break;
            end
        end

        erroRegime_pct = abs(valorFinal - spAlvo) / spAlvo * 100;

        linha = table(spAlvo, overshoot_pct, tempoSubida, tempoAcomodacao, erroRegime_pct, ...
            'VariableNames', {'SP_V','Overshoot_pct','TempoSubida_s','TempoAcomodacao_s','ErroRegime_pct'});
        resultados = [resultados; linha];
    end
end
