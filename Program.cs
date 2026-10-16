using System;
using System.IO;
using System.Text;
using System.Windows.Forms;
using System.Drawing;
using System.Linq;

namespace MahachiHamiltonSolver
{
    internal static class Program
    {
        [STAThread]
        static void Main()
        {
            ApplicationConfiguration.Initialize();
            Application.Run(new MahachiAlgorithmForm());
        }
    }

    public class MahachiAlgorithmForm : Form
    {
        private TextBox[] txtCoords = new TextBox[6];
        private TextBox[] txtDims = new TextBox[5];
        private TextBox txtMaxV, txtPath;
        private Button btnGo;

        public MahachiAlgorithmForm()
        {
            this.Text = "The Mahachi Algorithm - 5 DoF Engine";
            this.Size = new Size(460, 640);
            this.StartPosition = FormStartPosition.CenterScreen;
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;

            Label lblH = new Label { Text = "Mahachi Path Optimisation Algorithm", Font = new Font("Arial", 13, FontStyle.Bold), Location = new Point(20, 15), Size = new Size(400, 25) };
            this.Controls.Add(lblH);

            Label lblL = new Label { Text = "Robot Link Lengths L1 to L5 (meters):", Font = new Font("Arial", 9, FontStyle.Bold), Location = new Point(20, 55), Size = new Size(400, 20) };
            this.Controls.Add(lblL);

            string[] defaultDims = new string[] { "0.5", "0.4", "0.4", "0.3", "0.2" };
            for (int i = 0; i < 5; i++)
            {
                txtDims[i] = new TextBox { Text = defaultDims[i], Location = new Point(20 + (i * 80), 80), Size = new Size(70, 25) };
                this.Controls.Add(txtDims[i]);
            }

            Label lblV = new Label { Text = "Max Joint Velocity Limit (Deg/Sec):", Font = new Font("Arial", 9, FontStyle.Bold), Location = new Point(20, 125), Size = new Size(400, 20) };
            txtMaxV = new TextBox { Text = "45.0", Location = new Point(20, 150), Size = new Size(400, 25) };
            this.Controls.Add(lblV); this.Controls.Add(txtMaxV);

            Label lblPts = new Label { Text = "Initial (X,Y,Z) and Final (X,Y,Z) Task Locations (meters):", Font = new Font("Arial", 9, FontStyle.Bold), Location = new Point(20, 195), Size = new Size(400, 20) };
            this.Controls.Add(lblPts);

            string[] defaultCoords = new string[] { "0.3", "0.3", "0.4", "0.5", "-0.2", "0.2" };
            for (int i = 0; i < 6; i++)
            {
                int yPos = i < 3 ? 220 : 265;
                int xPos = 20 + ((i % 3) * 125);
                txtCoords[i] = new TextBox { Text = defaultCoords[i], Location = new Point(xPos, yPos), Size = new Size(110, 25) };
                this.Controls.Add(txtCoords[i]);
            }

            Label lblP = new Label { Text = "RoboAnalyzer Output Location (.csv):", Font = new Font("Arial", 9, FontStyle.Bold), Location = new Point(20, 315), Size = new Size(400, 20) };
            txtPath = new TextBox { Text = @"C:\RoboAnalyzer\MahachiTrajectory.csv", Location = new Point(20, 340), Size = new Size(400, 25) };
            this.Controls.Add(lblP); this.Controls.Add(txtPath);

            // FIXED LINE: Changed ForegroundColor to ForeColor to fix the compile error
            btnGo = new Button { Text = "Get Path", Location = new Point(20, 420), Size = new Size(400, 50), Font = new Font("Arial", 11, FontStyle.Bold), BackColor = Color.FromArgb(41, 128, 185), ForeColor = Color.White, FlatStyle = FlatStyle.Flat };
            btnGo.Click += BtnGo_Click;
            this.Controls.Add(btnGo);
        }

        private void BtnGo_Click(object sender, EventArgs e)
        {
            try
            {
                double[] L = txtDims.Select(t => double.Parse(t.Text)).ToArray();
                double[] C = txtCoords.Select(t => double.Parse(t.Text)).ToArray();
                double maxV = double.Parse(txtMaxV.Text);
                double dt = 0.1;

                if (L.Any(l => l <= 0) || maxV <= 0)
                {
                    MessageBox.Show("Dimensions and velocity bounds must be positive numbers.", "Validation Error");
                    return;
                }

                double reach = L[1] + L[2] + L[3] + L[4];
                double distInit = Math.Sqrt(C[0] * C[0] + C[1] * C[1] + Math.Pow(C[2] - L[0], 2));
                double distFinal = Math.Sqrt(C[3] * C[3] + C[4] * C[4] + Math.Pow(C[5] - L[0], 2));

                if (distInit > reach || distFinal > reach)
                {
                    MessageBox.Show("Workspace Reach Limit Violation: Target points reside outside physical bounds.", "Workspace Exception");
                    return;
                }

                double[] qInit = MahachiIK(C[0], C[1], C[2], L);
                double[] qFinal = MahachiIK(C[3], C[4], C[5], L);

                double maxDisp = qInit.Select((val, idx) => Math.Abs(qFinal[idx] - val)).Max();
                double tFinal = Math.Max(0.5, 1.5 * (maxDisp / maxV));

                string dir = Path.GetDirectoryName(txtPath.Text);
                if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir)) Directory.CreateDirectory(dir);

                StringBuilder sb = new StringBuilder();
                int totalSteps = (int)(tFinal / dt);

                double[] a2 = qInit.Select((val, idx) => (3.0 * (qFinal[idx] - val)) / Math.Pow(tFinal, 2)).ToArray();
                double[] a3 = qInit.Select((val, idx) => (-2.0 * (qFinal[idx] - val)) / Math.Pow(tFinal, 3)).ToArray();

                for (int step = 0; step <= totalSteps; step++)
                {
                    double t = step * dt;
                    string[] row = new string[5];
                    for (int j = 0; j < 5; j++)
                    {
                        row[j] = (qInit[j] + (a2[j] * t * t) + (a3[j] * t * t * t)).ToString("F4");
                    }
                    sb.AppendLine(string.Join(",", row));
                }

                File.WriteAllText(txtPath.Text, sb.ToString());
                MessageBox.Show("The Mahachi Path Optimization Complete!\nOptimal Duration Time: " + tFinal.ToString("F2") + " seconds.\nMatrix logged successfully for RoboAnalyzer.", "Success");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Execution Engine Processing Fault: " + ex.Message, "System Error");
            }
        }

        private double[] MahachiIK(double x, double y, double z, double[] L)
        {
            double j1 = Math.Atan2(y, x) * (180.0 / Math.PI);
            double r = Math.Sqrt(x * x + y * y);
            double ez = z - L[0];
            double d = Math.Sqrt(r * r + ez * ez);
            double l23 = L[1] + L[2];
            double l45 = L[3] + L[4];

            if (d > (l23 + l45)) d = l23 + l45;
            if (d < Math.Abs(l23 - l45)) d = Math.Abs(l23 - l45);

            double cosAlpha = (l23 * l23 + d * d - l45 * l45) / (2.0 * l23 * d);
            double alpha = Math.Acos(Math.Max(-1.0, Math.Min(1.0, cosAlpha))) * (180.0 / Math.PI);
            double beta = Math.Atan2(ez, r) * (180.0 / Math.PI);

            return new double[] { j1, beta + alpha, -alpha * 0.5, -alpha * 0.3, 0.0 };
        }
    }
}