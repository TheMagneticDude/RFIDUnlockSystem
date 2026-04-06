import discord
from discord.ext import commands
from discord import app_commands
import os

#========================= Constants =========================
#will fill in after to avoid secret being in repo
botToken = '';
#grab token from local env variable
botToken = os.getenv("DISCORD_TOKEN");

GUILD_ID = discord.Object(id=1490450047085838446);



 


#========================= Class =========================
class Client(commands.Bot):
    async def on_ready(self):
        print(f'Logged on as {self.user}!');
        try:
            synced = await self.tree.sync(guild=GUILD_ID)
            print(f"Synced {len(synced)} command(s) to guild.")
        except Exception as e:
            print(e)
        
    async def on_message(self, message):
        if message.author == self.user:
            return

        if message.content.startswith('!sesame'):
            await message.channel.send(
                f'Command Recieved: {message.author} executed: {message.content}'
            )

        await self.process_commands(message)



 




unlocked = False;

embedColor = discord.Color.red();
embedTitle = "Door State: Locked 🔒"
#discord.Color.green();
#"Door State: Unlocked 🔓"


def build_door_embed():
    global unlocked
    if unlocked:
        return discord.Embed(title="Door State: Unlocked 🔓",description="Door is currently unlocked",color=discord.Color.green())
    else:
        return discord.Embed(title="Door State: Locked 🔒",description="Door is currently locked",color=discord.Color.red())
        
        

#discord UI button component
class ViewButton(discord.ui.View):
    
    @discord.ui.button(label="Unlock", style=discord.ButtonStyle.primary, emoji="🧲")
    async def button_callback(self, interaction: discord.Interaction, button: discord.ui.Button):
        global unlocked
        #update embed on button press
        unlocked = not unlocked;
       
        #update message
        await interaction.response.edit_message(embed=doorembed, view=self);
 
    
    
    

#========================= Init =========================
intents = discord.Intents.default();
intents.message_content = True;
client = Client(command_prefix = "!sesame", intents = intents);





#========================= Commands =========================
@client.tree.command(name="test", description="test", guild = GUILD_ID)
async def test(interaction: discord.Interaction):
    await interaction.response.send_message("Test");
    
@client.tree.command(name="door", description="Controls Door", guild = GUILD_ID)
async def doorCommand(interaction: discord.Interaction):
    await interaction.response.send_message(embed = build_door_embed(), view=ViewButton());














#run bot
client.run(botToken);
